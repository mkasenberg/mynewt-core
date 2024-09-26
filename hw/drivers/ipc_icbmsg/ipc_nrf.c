/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <errno.h>
#include <os/os.h>
#include <ipc/ipc.h>
#include <nrfx.h>

#define IPC_MAX_CHANS MYNEWT_VAL(IPC_NRF5340_CHANNELS)

struct ipc_channel {
    ipc_icbmsg_recv_cb cb;
    void *user_data;
};

static struct ipc_channel ipcs[IPC_MAX_CHANS];

static void
ipc_nrf_isr(void)
{
    uint32_t irq_pend;
    int i;

    os_trace_isr_enter();

    /* Handle only interrupts that were enabled */
    irq_pend = NRF_IPC->INTPEND & NRF_IPC->INTEN;

    for (i = 0; i < IPC_MAX_CHANS; i++) {
        if (irq_pend & (0x1UL << i)) {
            NRF_IPC->EVENTS_RECEIVE[i] = 0;
            ipcs[i].cb(i, ipcs[i].user_data);
        }
    }

    os_trace_isr_exit();
}

// /* Below is to unmangle comma separated GPIO pins from MYNEWT_VAL */
// #define _Args(...) __VA_ARGS__
// #define STRIP_PARENS(X) X
// #define UNMANGLE_MYNEWT_VAL(X) STRIP_PARENS(_Args X)

static void
ipc_nrf_init_ipc(void)
{
    int i;

    /* Enable IPC channels */
    for (i = 0; i < IPC_MAX_CHANS; i++) {
        NRF_IPC->SEND_CNF[i] = (0x01UL << i);
        NRF_IPC->RECEIVE_CNF[i] = 0;
    }

    NRF_IPC->INTENCLR = 0xFFFF;
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetVector(IPC_IRQn, (uint32_t)ipc_nrf_isr);
    NVIC_EnableIRQ(IPC_IRQn);
}

#if MYNEWT_VAL(MCU_APP_CORE)
void
ipc_icbmsg_init(void)
{
    int i;

// #if MYNEWT_VAL(IPC_NRF5340_NET_GPIO)

//     unsigned int gpios[] = { UNMANGLE_MYNEWT_VAL(MYNEWT_VAL(IPC_NRF5340_NET_GPIO)) };
//     NRF_GPIO_Type *nrf_gpio;
// #endif

    /* Make sure network core if off when we set up IPC */
    nrf_reset_network_force_off(NRF_RESET, true)

// #if MYNEWT_VAL(NRF5340_EMBED_NET_CORE)
//     /*
//      * Get network core image size and placement in application flash.
//      * Then pass those two values to ipc_shared data to be used
//      * by virtual flash driver on network side.
//      */
//     if (&_binary_net_core_img_end - &_binary_net_core_img_start > 32) {
//         ipc_shared->net_core_image_address = (void *)&_binary_net_core_img_start;
//         ipc_shared->net_core_image_size = &_binary_net_core_img_end - &_binary_net_core_img_start;
//         /*
//          * For compatibility with first version of vflash driver that stored image address and
//          * image size in NRF_IPC_S.GPMEM[0..1].
//          */
//         if (MYNEWT_VAL(IPC_NRF5340_PRE_TRUSTZONE_NETCORE_BOOT)) {
//             NRF_IPC_S->GPMEM[0] = (uint32_t)&_binary_net_core_img_start;
//             NRF_IPC_S->GPMEM[1] = &_binary_net_core_img_end - &_binary_net_core_img_start;
//         }
//     }
// #endif

    if (MYNEWT_VAL(MCU_APP_SECURE) && !MYNEWT_VAL(IPC_NRF5340_PRE_TRUSTZONE_NETCORE_BOOT)) {
        /*
         * When bootloader is secure and application is not all peripherals are
         * in unsecure mode. This is done by bootloader.
         * If application runs in secure mode IPC manually chooses to use unsecure version
         * so net core can always use same peripheral.
         */
        NRF_SPU->PERIPHID[42].PERM &= ~SPU_PERIPHID_PERM_SECATTR_Msk;
    }

// #if MYNEWT_VAL(IPC_NRF5340_NET_GPIO)
//     /* Configure GPIOs for Networking Core */
//     for (i = 0; i < ARRAY_SIZE(gpios); i++) {
//         nrf_gpio = HAL_GPIO_PORT(gpios[i]);
//         nrf_gpio->PIN_CNF[HAL_GPIO_INDEX(gpios[i])] =
//             GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
//     }
// #endif

    ipc_nrf_init_ipc();

    if (MYNEWT_VAL(MCU_APP_SECURE)) {
        /* this allows netcore to access appcore RAM */
        NRF_SPU_S->EXTDOMAIN[0].PERM = SPU_EXTDOMAIN_PERM_SECATTR_Secure << SPU_EXTDOMAIN_PERM_SECATTR_Pos;
    }
}

void
ipc_icbmsg_netcore_init(void)
{
    /* Start Network Core */
    nrf_reset_network_force_off(NRF_RESET, false);

    /*
     * Waits for NET core to start and init it's side of IPC.
     * It may take several seconds if there is net core
     * embedded image in the application flash.
     */
    // Zephyr has similar while loop 
    // while (ipc_shared->ipc_state != APP_AND_NET_RUNNING);
}
#endif

#if MYNEWT_VAL(MCU_NET_CORE)
void
ipc_icbmsg_init(void)
{
    ipc_nrf_init_ipc();
}

void
ipc_icbmsg_netcore_init(void)
{
    /*
     * If ipc_state is already APP_AND_NET_RUNNING it means that net core
     * restarted without app core involvement, notify app core about such
     * case.
     */
    /* TODO sync with APP core*/
    ipc_icbmsg_open(channel);
}
#endif

#if MYNEWT_VAL(MCU_APP_CORE)
void
ipc_icbmsg_reset(void)
{
    /* Make sure network core if off when we reset IPC */
    nrf_reset_network_force_off(NRF_RESET, true)

    /* Start Network Core */
    ipc_icbmsg_netcore_init();
}
#endif

void
ipc_icbmsg_set_recv_cb(int channel, ipc_icbmsg_recv_cb cb,
                       void *user_data)
{
    assert(channel < IPC_MAX_CHANS);

    if (cb) {
        assert(ipcs[channel].cb == NULL);

        ipcs[channel].cb = cb;
        ipcs[channel].user_data = user_data;
        NRF_IPC->RECEIVE_CNF[channel] = (0x1UL << channel);
        NRF_IPC->INTENSET = (0x1UL << channel);
    } else {
        NRF_IPC->INTENCLR = (0x1UL << channel);
        NRF_IPC->RECEIVE_CNF[channel] = 0;
        ipcs[channel].cb = NULL;
        ipcs[channel].user_data = NULL;
    }
}
