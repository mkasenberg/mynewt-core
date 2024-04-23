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

INT_VECTOR_STACK_TOP(__StackTop)
INT_VECTOR_RESET_HANDLER(Reset_Handler)
INT_VECTOR_NMI_HANDLER(NMI_Handler)
INT_VECTOR_HARDFAULT_HANDLER(HardFault_Handler)
INT_VECTOR_MEMMANAGE_HANDLER(MemManage_Handler)
INT_VECTOR_BUSFAULT_HANDLER(BusFault_Handler)
INT_VECTOR_USAGEFAULT_HANDLER(UsageFault_Handler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_SVC_HANDLER(SVC_Handler)
INT_VECTOR_DEBUGMON_HANDLER(DebugMon_Handler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_PENDSV_HANDLER(PendSV_Handler)
INT_VECTOR_SYSTICK_HANDLER(SysTick_Handler)
INT_VECTOR(WWDG_IRQHandler)
INT_VECTOR(PVD_IRQHandler)
INT_VECTOR(TAMP_STAMP_IRQHandler)
INT_VECTOR(RTC_WKUP_IRQHandler)
INT_VECTOR(FLASH_IRQHandler)
INT_VECTOR(RCC_IRQHandler)
INT_VECTOR(EXTI0_IRQHandler)
INT_VECTOR(EXTI1_IRQHandler)
INT_VECTOR(EXTI2_IRQHandler)
INT_VECTOR(EXTI3_IRQHandler)
INT_VECTOR(EXTI4_IRQHandler)
INT_VECTOR(DMA1_Stream0_IRQHandler)
INT_VECTOR(DMA1_Stream1_IRQHandler)
INT_VECTOR(DMA1_Stream2_IRQHandler)
INT_VECTOR(DMA1_Stream3_IRQHandler)
INT_VECTOR(DMA1_Stream4_IRQHandler)
INT_VECTOR(DMA1_Stream5_IRQHandler)
INT_VECTOR(DMA1_Stream6_IRQHandler)
INT_VECTOR(ADC_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(EXTI9_5_IRQHandler)
INT_VECTOR(TIM1_BRK_TIM9_IRQHandler)
INT_VECTOR(TIM1_UP_IRQHandler)
INT_VECTOR(TIM1_TRG_COM_TIM11_IRQHandler)
INT_VECTOR(TIM1_CC_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(I2C1_EV_IRQHandler)
INT_VECTOR(I2C1_ER_IRQHandler)
INT_VECTOR(I2C2_EV_IRQHandler)
INT_VECTOR(I2C2_ER_IRQHandler)
INT_VECTOR(SPI1_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR(USART1_IRQHandler)
INT_VECTOR(USART2_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR(EXTI15_10_IRQHandler)
INT_VECTOR(RTC_Alarm_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(DMA1_Stream7_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(TIM5_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(TIM6_DAC_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR(DMA2_Stream0_IRQHandler)
INT_VECTOR(DMA2_Stream1_IRQHandler)
INT_VECTOR(DMA2_Stream2_IRQHandler)
INT_VECTOR(DMA2_Stream3_IRQHandler)
INT_VECTOR(DMA2_Stream4_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(DMA2_Stream5_IRQHandler)
INT_VECTOR(DMA2_Stream6_IRQHandler)
INT_VECTOR(DMA2_Stream7_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(RNG_IRQHandler)
INT_VECTOR(FPU_IRQHandler)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR_UNUSED(0)
INT_VECTOR(FMPI2C1_EV_IRQHandler)
INT_VECTOR(FMPI2C1_ER_IRQHandler)
INT_VECTOR(LPTIM1_IRQHandler)
