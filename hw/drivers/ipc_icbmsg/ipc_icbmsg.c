/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ICBMsg backend.
 *
 * This is an IPC service backend that dynamically allocates buffers for data storage
 * and uses ICMsg to send references to them.
 *
 * Shared memory organization
 * --------------------------
 *
 * Single channel (RX or TX) of the shared memory is divided into two areas: ICMsg area
 * followed by Blocks area. ICMsg is used to send and receive short 3-byte messages.
 * The ICMsg messages are queued inside the ICSM area using PBUF format.
 * Blocks area is evenly divided into aligned blocks. Blocks are used to allocate
 * buffers containing actual data. Data buffers can span multiple blocks. The first block
 * starts with the size of the following data.
 *
 *  +------------+-------------+
 *  | ICMsg area | Blocks area |
 *  +------------+-------------+
 *       _______/               \_________________________________________
 *      /                                                                 \
 *      +-----------+-----------+-----------+-----------+-   -+-----------+
 *      |  Block 0  |  Block 1  |  Block 2  |  Block 3  | ... | Block N-1 |
 *      +-----------+-----------+-----------+-----------+-   -+-----------+
 *            _____/                                     \_____
 *           /                                                 \
 *           +------+--------------------------------+---------+
 *           | size | data_buffer[size] ...          | padding |
 *           +------+--------------------------------+---------+
 *
 * The sender holds information about reserved blocks using bitarray and it is responsible
 * for allocating and releasing the blocks. The receiver just tells the sender that it
 * does not need a specific buffer anymore.
 *
 * Control messages
 * ----------------
 *
 * ICMsg is used to send and receive small 3-byte control messages.
 *
 *  - Send data
 *    | MSG_DATA | endpoint address | block index |
 *    This message is used to send data buffer to specific endpoint.
 *
 *  - Release data
 *    | MSG_RELEASE_DATA | 0 | block index |
 *    This message is a response to the "Send data" message and it is used to inform that
 *    specific buffer is not used anymore and can be released. Endpoint addresses does
 *    not matter here, so it is zero.
 *
 *  - Bound endpoint
 *    | MSG_BOUND | endpoint address | block index |
 *    This message starts the bounding of the endpoint. The buffer contains a
 *    null-terminated endpoint name.
 *
 *  - Release bound endpoint
 *    | MSG_RELEASE_BOUND | endpoint address | block index |
 *    This message is a response to the "Bound endpoint" message and it is used to inform
 *    that a specific buffer (starting at "block index") is not used anymore and
 *    a the endpoint is bounded and can now receive a data.
 *
 * Bounding endpoints
 * ------------------
 *
 * When ICMsg is bounded and user registers an endpoint on initiator side, the backend
 * sends "Bound endpoint". Endpoint address is assigned by the initiator. When follower
 * gets the message and user on follower side also registered the same endpoint,
 * the backend calls "bound" callback and sends back "Release bound endpoint".
 * The follower saves the endpoint address. The follower's endpoint is ready to send
 * and receive data. When the initiator gets "Release bound endpoint" message or any
 * data messages, it calls bound endpoint and it is ready to send data.
 */


#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#if MYNEWT_VAL(MCU_APP_CORE)
#define TX_BLOCKS_NUM 16
#define RX_BLOCKS_NUM 24
else
#define TX_BLOCKS_NUM 24
#define RX_BLOCKS_NUM 16
#endif

#define tx_BLOCKS_NUM TX_BLOCKS_NUM
#define rx_BLOCKS_NUM RX_BLOCKS_NUM

/* A string used to synchronize cores */
static const uint8_t magic[] = {0x45, 0x6d, 0x31, 0x6c, 0x31, 0x4b,
                				0x30, 0x72, 0x6e, 0x33, 0x6c, 0x69, 0x34};

struct control_message {
    /* Message type. */
    uint8_t msg_type;
    /* Endpoint address or zero for MSG_RELEASE_DATA. */
    uint8_t ept_addr;
    /* Block index to send or release. */
    uint8_t block_index;
};

/** Allowed number of endpoints. */
#define NUM_EPT CONFIG_IPC_SERVICE_BACKEND_ICBMSG_NUM_EP

/** Special endpoint address indicating invalid (or empty) entry. */
#define EPT_ADDR_INVALID 0xFF

/** Special value for empty entry in bound message waiting table. */
#define WAITING_BOUND_MSG_EMPTY 0xFFFF

/** Size of the header (size field) of the block. */
#define BLOCK_HEADER_SIZE (sizeof(struct block_header))

/** Flag indicating that ICMsg was bounded for this instance. */
#define CONTROL_BOUNDED BIT(31)

/** Registered endpoints count mask in flags. */
#define FLAG_EPT_COUNT_MASK 0xFFFF

/**
 * @brief Value of @p x rounded up to the next multiple of @p align.
 */
#define ROUND_UP(x, align)                                  \
    ((((unsigned long)(x) + ((unsigned long)(align) - 1)) / \
      (unsigned long)(align)) * (unsigned long)(align))

/**
 * Number of bytes per each ICMsg message. It is used to calculate size of ICMsg area.
 */
#define BYTES_PER_ICMSG_MESSAGE (ROUND_UP(sizeof(struct control_message),      \
                                          sizeof(void *)) + PBUF_PACKET_LEN_SZ)

/**
 * Maximum ICMsg overhead. It is used to calculate size of ICMsg area.
 */
#define ICMSG_BUFFER_OVERHEAD(i) \
    (PBUF_HEADER_OVERHEAD(GET_CACHE_ALIGNMENT(i)) + 2 * BYTES_PER_ICMSG_MESSAGE)

/**
 * Returns required block alignment for instance "i".
 */
#define GET_CACHE_ALIGNMENT(i) (sizeof(uint32_t))

/**
 * Calculates minimum size required for ICMsg region for specific number of local
 * and remote blocks. The minimum size ensures that ICMsg queue is will never overflow
 * because it can hold data message for each local block and release message
 * for each remote block.
 */
#define GET_ICMSG_MIN_SIZE(i, local_blocks, remote_blocks) \
    (ICMSG_BUFFER_OVERHEAD(i) + BYTES_PER_ICMSG_MESSAGE *  \
     (local_blocks + remote_blocks))

/**
 * Calculate aligned block size by evenly dividing remaining space after removing
 * the space for ICMsg.
 */
#define GET_BLOCK_SIZE(i, total_size, local_blocks, remote_blocks) ROUND_DOWN( \
    ((total_size) - GET_ICMSG_MIN_SIZE(i, (local_lbocks), (remote_blocks))) /  \
    (local_blocks), GET_CACHE_ALIGNMENT(i))

/**
 * Calculate offset where area for blocks starts which is just after the ICMsg.
 */
#define GET_BLOCKS_OFFSET(i, total_size, local_blocks, remote_blocks) \
    ((total_size) - GET_BLOCK_SIZE(i, (total_size), (local_blocks),   \
                                   (remote_blocks)) * (local_blocks))

/**
 * Return shared memory start address aligned to block alignment and cache line.
 */
#define GET_MEM_ADDR_INST(i, direction)                               \
    ROUND_UP(_binary_ipc##i##_##direction##_start, GET_CACHE_ALIGNMENT(i))

/**
 * Return shared memory end address aligned to block alignment and cache line.
 */
#define GET_MEM_END_INST(i, direction)                                \
    ROUND_DOWN(_binary_ipc##i##_##direction##_end, GET_CACHE_ALIGNMENT(i))

/**
 * Return shared memory size aligned to block alignment and cache line.
 */
#define GET_MEM_SIZE_INST(i, direction)                               \
	(GET_MEM_END_INST(i, direction) - GET_MEM_ADDR_INST(i, direction))

/**
 * Returns GET_ICMSG_SIZE, but for specific instance and direction.
 * 'loc' and 'rem' parameters tells the direction. They can be either "tx, rx"
 *  or "rx, tx".
 */
#define GET_ICMSG_SIZE_INST(i, loc, rem)         \
    GET_BLOCKS_OFFSET(i,                         \
                      GET_MEM_SIZE_INST(i, loc), \
                      loc##_BLOCKS_NUM,          \
                      rem##_BLOCKS_NUM)

/**
 * Returns address where area for blocks starts for specific instance and direction.
 * 'loc' and 'rem' parameters tells the direction. They can be either "tx, rx"
 *  or "rx, tx".
 */
#define GET_BLOCKS_ADDR_INST(i, loc, rem) \
    GET_MEM_ADDR_INST(i, loc) +           \
    GET_ICMSG_SIZE_INST(i, loc, rem)

/**
 * Returns block size for specific instance and direction.
 * 'loc' and 'rem' parameters tells the direction. They can be either "tx, rx"
 *  or "rx, tx".
 */
#define GET_BLOCK_SIZE_INST(i, loc, rem)      \
    GET_BLOCK_SIZE(i,                         \
                   GET_MEM_SIZE_INST(i, loc), \
                   loc##_BLOCKS_NUM,          \
                   rem##_BLOCKS_NUM)

enum msg_type {
    /* Data message. */
    MSG_DATA = 0,
    /* Release data buffer message. */
    MSG_RELEASE_DATA,
    /* Endpoint bounding message. */
    MSG_BOUND,
    /* Release endpoint bound message.
     * This message is also indicator for the receiving side
     * that the endpoint bounding was fully processed on
     * the sender side.
     */
    MSG_RELEASE_BOUND,
};

enum ept_bounding_state {
    /* Endpoint in not configured (initial state). */
    EPT_UNCONFIGURED = 0,
    /* Endpoint is configured, waiting for work queue to
     * start bounding process.
     */
    EPT_CONFIGURED,
    /* Only on initiator. Bound message was send,
     * but bound callback was not called yet, because
     * we are waiting for any incoming messages.
     */
    EPT_BOUNDING,
    /* Bounding is done. Bound callback was called. */
    EPT_READY,
};

struct channel_config {
    /* Address where the blocks start. */
    uint8_t *blocks_ptr;
    /* Size of one block. */
    size_t block_size;
    /* Number of blocks. */
    size_t block_count;
};

struct ept_data {
    /* Bounding state. */
    atomic_t state;
    /* Endpoint address. */
    uint8_t addr;
    /* Device name, unused on our side */
    char name[5];
};

struct ipc_instance {
    /* Bit is set when TX block is in use */
    uint8_t tx_usage_bitmap[DIV_ROUND_UP(TX_BLOCKS_NUM, 8)];
    /* Bit is set, if the buffer starting at
     * this block should be kept after exit
     * from receive handler.
     */
    uint8_t rx_usage_bitmap[DIV_ROUND_UP(RX_BLOCKS_NUM, 8)];
    /* TX ICSMsg Area packet buffer */
    struct pbuf_cfg tx_pb;
    /* RX ICSMsg Area packet buffer */
    struct pbuf_cfg rx_pb;
    /* TX channel config. */
    struct channel_config tx;
    /* RX channel config. */
    struct channel_config rx;
    /* Array of registered endpoints. */
    struct ept_data ept[NUM_EPT];
    /* The bound messages waiting to be registered. */
    uint16_t waiting_bound[NUM_EPT];
    /* Flags on higher bits, number of registered
     * endpoints on lower.
     */
    uint32_t flags;
    /* This side has an initiator role. */
    bool is_initiator;
};

struct block_header {
    /* Size of the data field. It must be volatile, because
     * when this value is read and validated for security
     * reasons, compiler cannot generate code that reads
     * it again after validation.
     */
    volatile size_t size;
};

struct block_content {
    struct block_header header;
    /* Buffer data. */
    uint8_t data[];
};

struct control_message {
    /* Message type. */
    uint8_t msg_type;
    /* Endpoint address or zero for MSG_RELEASE_DATA. */
    uint8_t ept_addr;
    /* Block index to send or release. */
    uint8_t block_index;
};

static struct ipc_instance ipc_instances[1] = {
    {
        .tx_pb = PBUF_CFG_INIT(GET_MEM_ADDR_INST(i, tx),
                               GET_ICMSG_SIZE_INST(i, tx, rx),
                               GET_CACHE_ALIGNMENT(i)),
        .rx_pb = PBUF_CFG_INIT(GET_MEM_ADDR_INST(i, tx),
                               GET_ICMSG_SIZE_INST(i, tx, rx),
                               GET_CACHE_ALIGNMENT(i)),
        .tx = {
            .blocks_ptr = (uint8_t *)GET_BLOCKS_ADDR_INST(i, tx, rx),
            .block_count = TX_BLOCKS_NUM,
            .block_size = GET_BLOCK_SIZE_INST(i, tx, rx),
        },
        .rx = {
            .blocks_ptr = (uint8_t *)GET_BLOCKS_ADDR_INST(i, rx, tx),
            .block_count = RX_BLOCKS_NUM,
            .block_size = GET_BLOCK_SIZE_INST(i, rx, tx),
        },
    },
};

#define IPC_MAX_CHANS MYNEWT_VAL(IPC_NRF5340_CHANNELS)
static struct ipc_instance *channel_to_ipc[IPC_MAX_CHANS];
static uint8_t channel_to_ept[IPC_MAX_CHANS];

/**
 * Calculate pointer to block from its index and channel configuration (RX or TX).
 * No validation is performed.
 */
static struct block_content *
block_from_index(const struct channel_config *ch_conf, size_t block_index)
{
    return (struct block_content *)(ch_conf->blocks_ptr + block_index * ch_conf->block_size);
}

/**
 * Calculate pointer to data buffer from block index and channel configuration (RX or TX).
 * Also validate the index and optionally the buffer size allocated on the this block.
 *
 * @param[in]  ch_conf        The channel
 * @param[in]  block_index    Block index
 * @param[out] size           Size of the buffer allocated on the block if not NULL.
 *                            The size is also checked if it fits in the blocks area.
 *                            If it is NULL, no size validation is performed.
 * @param[in]  invalidate_cache  If size is not NULL, invalidates cache for entire buffer
 *                               (all blocks). Otherwise, it is ignored.
 * @return    Pointer to data buffer or NULL if validation failed.
 */
static uint8_t *
buffer_from_index_validate(const struct channel_config *ch_conf,
                           size_t block_index, size_t *size,
                           bool invalidate_cache)
{
    size_t allocable_size;
    size_t buffer_size;
    uint8_t *end_ptr;
    struct block_content *block;

    if (block_index >= ch_conf->block_count) {
        /* Block index invalid */
        return NULL;
    }

    block = block_from_index(ch_conf, block_index);

    if (size != NULL) {
        if (invalidate_cache) {
            sys_cache_data_invd_range(block, BLOCK_HEADER_SIZE);
        }
        allocable_size = ch_conf->block_count * ch_conf->block_size;
        end_ptr = ch_conf->blocks_ptr + allocable_size;
        buffer_size = block->header.size;

        if ((buffer_size > allocable_size - BLOCK_HEADER_SIZE) ||
            (&block->data[buffer_size] > end_ptr)) {
            /* Block corrupted */
            return NULL;
        }

        *size = buffer_size;
        if (invalidate_cache) {
            sys_cache_data_invd_range(block->data, buffer_size);
        }
    }

    return block->data;
}

/**
 * Calculate block index based on data buffer pointer and validate it.
 *
 * @param[in]  ch_conf  The channel
 * @param[in]  buffer   Pointer to data buffer
 * @param[out] size     Size of the allocated buffer if not NULL.
 *                      The size is also checked if it fits in the blocks area.
 *                      If it is NULL, no size validation is performed.
 * @return  lock index or negative error code
 * @retval -EINVAL    The buffer is not correct
 */
static int
buffer_to_index_validate(const struct channel_config *ch_conf,
                         const uint8_t *buffer, size_t *size)
{
    size_t block_index;
    uint8_t *expected;

    block_index = (buffer - ch_conf->blocks_ptr) / ch_conf->block_size;

    expected = buffer_from_index_validate(ch_conf, block_index, size, false);

    if (expected == NULL || expected != buffer) {
        /* Pointer invalid */
        return -EINVAL;
    }

    return block_index;
}

static int
find_zero_bits(uint8_t bitmap[], size_t total_bits,
               size_t n, size_t *start_index)
{
    size_t zero_count = 0;
    size_t first_zero_bit_pos;
    size_t bit_id;
    size_t byte_id;
    uint8_t bit_pos;

    /* Find the first sequence of n consecutive 0 bits */
    for (size_t bit_id = 0; bit_id < total_bits; ++bit_id) {
        byte_id = bit_id / 8;
        bit_pos = bit_id % 8;

        if ((bitmap[byte_id] & (1 << bit_pos)) == 0) {
            if (zero_count == 0) {
                first_zero_bit_pos = bit_id;
            }
            zero_count++;

            if (zero_count == n) {
                *start_index = first_zero_bit_pos;
                return 0;
            }
        } else {
            zero_count = 0;
        }
    }

    return -1;
}

static void
alloc_zero_bits(uint8_t bitmap[], size_t n, size_t start_index) {
    for (size_t i = 0; i < n; ++i) {
        size_t bit_index = start_index + i;
        size_t byte_index = bit_index / 8;
        size_t bit_pos = bit_index % 8;

        bitmap[byte_index] |= (1 << bit_pos);
    }
}

/**
 * Allocate buffer for transmission
 *
 * @param[in,out] size    Required size of the buffer. If zero, first available block is
 *            allocated and all subsequent available blocks. Size actually
 *            allocated which is not less than requested.
 * @param[out] buffer    Allocated buffer data.
 * @param[in] timeout    Timeout.
 *
 * @return        Positive index of the first allocated block or negative error.
 * @retval -EINVAL    If requested size is bigger than entire allocable space.
 * @retval -ENOSPC    If timeout was K_NO_WAIT and there was not enough space.
 * @retval -EAGAIN    If timeout occurred.
 */
static int
alloc_tx_buffer(int channel, uint32_t size, uint8_t **buffer,
                uint8_t *tx_block_index)
{
    struct ipc_instance *ipc = channel_to_ipc[channel];
    struct block_content *block;
    int total_bits = BITMAP_SIZE * 8;
    size_t total_size = size + BLOCK_HEADER_SIZE;
    size_t num_blocks = DIV_ROUND_UP(total_size, ipc->tx.block_size);

    rc = find_zero_bits(ipc->tx_usage_bitmap, total_bits,
                        num_blocks, tx_block_index);
    if (rc) {
        return rc;
    }

    alloc_zero_bits(ipc->tx_usage_bitmap, num_blocks, *tx_block_index) ;

    /* Get block pointer and adjust size to actually allocated space. */
    size = ipc->tx.block_size * num_blocks - BLOCK_HEADER_SIZE;
    block = block_from_index(&ipc->tx, *tx_block_index);
    block->header.size = size;
    *buffer = block->data;

    return 0;
}

int
ipc_icbmsg_alloc_tx_buf(int channel, struct ipc_icmsg_buf *buf, uint32_t size)
{
    return alloc_tx_buffer(channel, size, &buf->data, &buf->block_id);
}

/**
 * Release all or part of the blocks occupied by the buffer.
 *
 * @param[in] tx_block_index    First block index to release, no validation is performed,
 *                so caller is responsible for passing valid index.
 * @param[in] size        Size of data buffer, no validation is performed,
 *                so caller is responsible for passing valid size.
 * @param[in] new_size        If less than zero, release all blocks, otherwise reduce
 *                size to this value and update size in block header.
 *
 * @returns        Positive block index where the buffer starts or negative error.
 * @retval -EINVAL    If invalid buffer was provided or size is greater than already
 *            allocated size.
 */
static int
release_tx_blocks(struct ipc_instance *ipc, size_t tx_block_index,
                  size_t size, int new_size)
{
    struct block_content *block;
    size_t num_blocks;
    size_t total_size;
    size_t new_total_size;
    size_t new_num_blocks;
    size_t release_index;
    int r;

    /* Calculate number of blocks. */
    total_size = size + BLOCK_HEADER_SIZE;
    num_blocks = DIV_ROUND_UP(total_size, ipc->tx.block_size);

    if (new_size >= 0) {
        /* Calculate and validate new values. */
        new_total_size = new_size + BLOCK_HEADER_SIZE;
        new_num_blocks = DIV_ROUND_UP(new_total_size, ipc->tx.block_size);
        if (new_num_blocks > num_blocks) {
            return -EINVAL;
        }
        /* Update actual buffer size and number of blocks to release. */
        block = block_from_index(&ipc->tx, tx_block_index);
        block->header.size = new_size;
        release_index = tx_block_index + new_num_blocks;
        num_blocks = num_blocks - new_num_blocks;
    } else {
        /* If size is negative, release all blocks. */
        release_index = tx_block_index;
    }

    if (num_blocks > 0) {
        /* Free bits in the bitmap. */
        r = sys_bitarray_free(ipc->tx_usage_bitmap, num_blocks,
                      release_index);
        if (r < 0) {
            return r;
        }
    }

    return tx_block_index;
}

static int
send_control_message(int channel, enum msg_type msg_type, uint8_t block_index)
{
    int ret;
    struct ipc_instance *ipc = channel_to_ipc[channel];
    // uint8_t ept_addr = channel_to_ept[channel];

    const struct control_message message = {
        .msg_type = (uint8_t)msg_type,
        .ept_addr = ept_addr,
        .block_index = block_index,
    };

    if (ipc->state != ICMSG_STATE_READY) {
        return -EBUSY;
    }

    ret = pbuf_write(ipc->tx_pb, message, sizeof(message));

    if (ret < 0) {
        return ret;
    } else if (ret < len) {
        return -EBADMSG;
    }

    nrfx_ipc_signal(channel);

    return ret;
}

/**
 * Release received buffer. This function will just send release control message.
 *
 * @param[in] buffer      Buffer to release.
 * @param[in] msg_type    Message type: MSG_RELEASE_BOUND or MSG_RELEASE_DATA.
 *
 * @return                zero or ICMsg send error.
 */
static int
send_release(int channel, const uint8_t *buffer, enum msg_type msg_type)
{
    int rx_block_index;
    struct ipc_instance *ipc = channel_to_ipc[channel];

    rx_block_index = buffer_to_index_validate(&ipc->rx, buffer, NULL);
    if (rx_block_index < 0) {
        return rx_block_index;
    }

    return send_control_message(channel, msg_type, rx_block_index);
}

/**
 * Send data contained in specified block. It will adjust data size and flush cache
 * if necessary. If sending failed, allocated blocks will be released.
 *
 * @param[in] msg_type          Message type: MSG_BOUND or MSG_DATA.
 * @param[in] ept_addr          Endpoints address.
 * @param[in] tx_block_index    Index of first block containing data, it is not validated,
 *                              so caller is responsible for passing only valid index.
 * @param[in] size              Actual size of the data, can be smaller than allocated,
 *                              but it cannot change number of required blocks.
 *
 * @return                      number of bytes sent in the message or negative error code.
 */
static int
send_block(int channel, enum msg_type msg_type, size_t tx_block_index, size_t size)
{
    int r;
    struct ipc_instance *ipc = channel_to_ipc[channel];
    struct block_content *block;

    block = block_from_index(&ipc->tx, tx_block_index);
    block->header.size = size;

    r = send_control_message(channel, msg_type, tx_block_index);
    if (r < 0) {
        release_tx_blocks(channel, tx_block_index, size, -1);
    }

    return r;
}

/**
 * Find registered endpoint that matches given "bound endpoint" message. When found,
 * the "release bound endpoint" message is send.
 *
 * @param[in] rx_block_index    Block containing the "bound endpoint" message.
 * @param[in] ept_addr        Endpoint address.
 *
 * @return    negative error code or non-negative search result.
 * @retval 0    match not found.
 * @retval 1    match found and processing was successful.
 */
static int
match_bound_msg(struct ipc_instance *ipc, size_t rx_block_index, uint8_t ept_addr)
{
    struct block_content *block;
    uint8_t *buffer;
    int ept_index;
    struct ept_data *ept;
    int r;
    bool valid_state;

    /* Find endpoint that matches requested name. */
    block = block_from_index(&ipc->rx, rx_block_index);
    buffer = block->data;


    for (i = 0; i < NUM_EPT; i++) {
        ept = &ipc->ept[i];
        if (atomic_get(&ept->state) == EPT_CONFIGURED &&
            strncmp(ept->cfg->name, name, name_size) == 0) {
            return i;
        }
    }

    /* Set endpoint address and mapping. Move it to "ready" state. */
    ept = &ipc->ept[ept_index];
    ept->addr = ept_addr;

    assert(ept->state == EPT_CONFIGURED);
    ept->state = EPT_READY;

    /* Release the bound message and inform remote that we are ready to receive. */
    r = send_release(ipc, buffer, MSG_RELEASE_BOUND, ept_addr);
    if (r < 0) {
        return r;
    }

    return 1;
}

static int
send_bound_message(int channel)
{
    int rc;
    size_t msg_len;
    uint8_t *buffer;
    size_t tx_block_index;
    struct ipc_instance *ipc = channel_to_ipc[channel];
    struct ept_data *ept = ipc->ept[channel_to_ept[channel]];

    msg_len = strlen(ept->name) + 1;
    rc = alloc_tx_buffer(channel, msg_len, &buffer, &tx_block_index);
    if (rc >= 0) {
        strcpy(buffer, ept->name);
        rc = send_block(channel, MSG_BOUND, tx_block_index, msg_len);
    }

    return rc;
}

static void
ept_bound_process(int channel)
{
    struct ept_data *ept = NULL;
    size_t i;
    int r = 0;

    /* Skip processing if ICMsg was not bounded yet. */
    if (!(ipc->flags & CONTROL_BOUNDED)) {
        return;
    }

    if (ipc->is_initiator) {
        /* Initiator just sends bound message after endpoint was registered. */
        for (i = 0; i < NUM_EPT; i++) {
            ept = &ipc->ept[i];

            if (ept->state == EPT_CONFIGURED) {
                ept->state = EPT_BOUNDING;

                r = send_bound_message(channel, ept);
                if (r < 0) {
                    /* Failed to send bound */
                    ept->state = EPT_UNCONFIGURED;
                }
            }
        }
    } else {
        /* Walk over all waiting bound messages and match to local endpoints. */
        for (i = 0; i < NUM_EPT; i++) {
            if (ipc->waiting_bound[i] != WAITING_BOUND_MSG_EMPTY) {
                r = match_bound_msg(ipc, ipc->waiting_bound[i], i);

                if (r != 0) {
                    ipc->waiting_bound[i] = WAITING_BOUND_MSG_EMPTY;
                }
            }
        }
    }
}

static struct ept_data *
get_ept_and_rx_validate(struct ipc_instance *ipc, uint8_t ept_addr)
{
    struct ept_data *ept;

    if (ept_addr >= NUM_EPT || ipc->ept_map[ept_addr] >= NUM_EPT) {
        return NULL;
    }

    ept = &ipc->ept[ipc->ept_map[ept_addr]];

    if (ept->state == EPT_READY) {
        /* Valid state - nothing to do. */
    } else if (ept->state == EPT_BOUNDING) {
        /* Endpoint bound callback was not called yet - call it. */
        ept->state = EPT_READY;
        // if (ept->cfg->cb.bound != NULL) {
        //     ept->cfg->cb.bound(ept->cfg->priv);
        // }
    } else {
        return NULL;
    }

    return ept;
}

int
ipc_icbmsg_send_buf(int channel, struct ipc_icmsg_buf *buf)
{
    /* Send data message. */
    return send_block(channel, MSG_DATA, buf->block_id, buf->len);
}

static int
received_release_data(struct ipc_instance *ipc, size_t tx_block_index)
{
    uint8_t *buffer;
    size_t size;
    int r;

    /* Validate. */
    buffer = buffer_from_index_validate(&ipc->tx, tx_block_index, &size, false);
    if (buffer == NULL) {
        return -EINVAL;
    }

    /* Release. */
    r = release_tx_blocks(ipc, tx_block_index, size, -1);
    if (r < 0) {
        return r;
    }

    return r;
}

static int
received_bound(struct ipc_instance *ipc, size_t rx_block_index, uint8_t ept_addr)
{
    size_t size;
    uint8_t *buffer;

    /* Validate */
    buffer = buffer_from_index_validate(&ipc->rx, rx_block_index, &size, true);
    if (buffer == NULL) {
        /* Received invalid block index */
        return -1;
    }

    /* Put message to waiting array. */
    ipc->waiting_bound[ept_addr] = rx_block_index;

    ept_bound_process(channel);

    return 0;
}

/**
 * Data message received.
 */
static int
received_data(struct ipc_instance *ipc, size_t rx_block_index, uint8_t ept_addr)
{
    uint8_t *buffer;
    struct ept_data *ept;
    size_t size;
    int bit_val;

    /* Validate. */
    buffer = buffer_from_index_validate(&ipc->rx, rx_block_index, &size, true);
    ept = get_ept_and_rx_validate(ipc, ept_addr);
    if (buffer == NULL || ept == NULL) {
        return -EINVAL;
    }

    /* Clear bit. If cleared, specific block will not be hold after the callback. */
    sys_bitarray_clear_bit(ipc->rx_hold_bitmap, rx_block_index);

    /* Call the endpoint callback. It can set the hold bit. */
    ept->cfg->cb.received(buffer, size, ept->cfg->priv);

    /* If the bit is still cleared, request release of the buffer. */
    sys_bitarray_test_bit(ipc->rx_hold_bitmap, rx_block_index, &bit_val);
    if (!bit_val) {
        send_release(ipc, buffer, MSG_RELEASE_DATA, 0);
    }

    return 0;
}

/**
 * Callback called by ICMsg that handles message (data or endpoint bound) received
 * from the remote.
 *
 * @param[in] data    Message received from the ICMsg.
 * @param[in] len    Number of bytes of data.
 * @param[in] priv    Opaque pointer to device instance.
 */
static void
control_received(const void *data, size_t len, void *priv)
{
    int channel = *priv;
    struct ipc_instance *ipc = channel_to_ipc[channel];
    const struct control_message *message = (const struct control_message *)data;
    struct ept_data *ept;
    uint8_t ept_addr;
    int r = 0;

    /* Allow messages longer than 3 bytes, e.g. for future protocol versions. */
    if (len < sizeof(struct control_message)) {
        r = -EINVAL;
        return;
    }

    ept_addr = message->ept_addr;
    if (ept_addr >= NUM_EPT) {
        r = -EINVAL;
        return;
    }

    switch (message->msg_type) {
    case MSG_RELEASE_DATA:
        r = received_release_data(ipc, message->block_index);
        break;
    case MSG_RELEASE_BOUND:
        r = received_release_data(ipc, message->block_index);
        if (r >= 0) {
            ept = get_ept_and_rx_validate(ipc, ept_addr);
            if (ept == NULL) {
                r = -EINVAL;
            }
        }
        break;
    case MSG_BOUND:
        r = received_bound(ipc, message->block_index, ept_addr);
        break;
    case MSG_DATA:
        r = received_data(ipc, message->block_index, ept_addr);
        break;
    default:
        /* Silently ignore other messages types. They can be used in future
         * protocol version.
         */
        break;
    }
}

static uint32_t
data_available(int channel)
{
    struct ipc_instance *ipc = channel_to_ipc[channel];

    return pbuf_read(ipc->rx_pb, NULL, 0);
}

void
ipc_icbmsg_process_icmsg(int channel)
{
    struct ipc_instance *ipc = channel_to_ipc[channel];
    uint8_t rx_buffer[CONFIG_PBUF_RX_READ_BUF_SIZE] __aligned(4);
    uint32_t state = dev_data->state;
    uint32_t len = data_available(channel);

    if (len == 0) {
        /* Unlikely, no data in buffer. */
        return;
    }

    assert(len <= sizeof(rx_buffer));

    if (sizeof(rx_buffer) < len) {
        return;
    }

    len = pbuf_read(ipc->rx_pb, rx_buffer, sizeof(rx_buffer));

    if (ipc->state == ICMSG_STATE_READY) {
        control_received(rx_buffer, len, dev_data->ctx);
    } else {
        assert(ipc->state == ICMSG_STATE_BUSY);

        /* Allow magic number longer than sizeof(magic) for future protocol version. */
        bool endpoint_invalid = (len < sizeof(magic) ||
                    memcmp(magic, rx_buffer, sizeof(magic)));

        assert(!endpoint_invalid);

        /* Set flag that ICMsg is bounded and now, endpoint bounding may start. */
        ipc->flags |= CONTROL_BOUNDED;
        ept_bound_process(channel);
        ipc->state = ICMSG_STATE_READY;
    }
}

int
ipc_icbmsg_send(int channel, const void *data, uint16_t len)
{
    int rc;
    uint8_t *buffer;
    size_t tx_block_index;

    /* Allocate the buffer. */
    rc = alloc_tx_buffer(channel, len, &buffer, &tx_block_index);
    if (rc < 0) {
        return rc;
    }

    /* Copy data to allocated buffer. */
    memcpy(buffer, data, len);

    /* Send data message. */
    rc = send_block(channel, MSG_DATA, tx_block_index, len);
    if (rc < 0) {
        return rc;
    }

    return 0;
}

int
ipc_icmsg_register_ept(int channel, int ipc_id)
{
    struct ipc_instance *ipc;
	struct ept_data *ept = NULL;
	uint8_t ept_addr;

    assert(channel < IPC_MAX_CHANS);
    assert(ipc_id < sizeof(ipc_instances));
    ipc = &ipc_instances[ipc_id];

	/* Reserve new endpoint index. */
	ept_addr = (++ipc->flags) & FLAG_EPT_COUNT_MASK;
	assert(ept_addr < NUM_EPT);

    channel_to_ept[channel] = ept_addr;
    channel_to_ipc[i] = ipc;

	/* Add new endpoint. */
	ept = &ipc->ept[ept_addr];
	if (ipc->is_initiator) {
		ept->addr = ept_addr;
	}
	ept->state = EPT_CONFIGURED;

	ept_bound_process(channel);

	return r;
}

int
ipc_icbmsg_open(int channel)
{
    int rc;
    struct ipc_instance *ipc = channel_to_ipc[channel];

    assert(ipc->state == ICMSG_STATE_OFF);
    ipc->state = ICMSG_STATE_BUSY;

    memset(ipc->tx_usage_bitmap, 0, TX_BLOCKS_NUM);
    memset(ipc->rx_usage_bitmap, 0, RX_BLOCKS_NUM);

    rc = pbuf_init(ipc->tx_pb);
    assert(rc == 0);

    /* Initialize local copies of rx_pb. */
    ipc->rx_pb->data.wr_idx = 0;
    ipc->rx_pb->data.rd_idx = 0;

    rc = pbuf_write(ipc->tx_pb, magic, sizeof(magic));
    assert(rc == 0);

    notify_process(channel);
}
