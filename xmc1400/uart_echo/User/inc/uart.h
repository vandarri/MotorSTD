/*
 * uart.h
 *
 *  Created on: Mar 7, 2021
 *      Author: HOME
 */

#ifndef USER_INC_UART_H_
#define USER_INC_UART_H_

#include <uart_conf.h>
#include <xmc_common.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_uart.h>

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
/*
 * @brief Represents the maximum data size for DMA transaction*/
#define UART_DMA_MAXCOUNT (4095U)

/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * @ingroup UART_enumerations
 * @{
 */

/**
 * @brief Enum to describe the possible status values, returned
 *  by UART APIs.
 */
typedef enum UART_STATUS
{
  UART_STATUS_SUCCESS,        /**< Indicates App initialization state successful */

  UART_STATUS_FAILURE,        /**< Unknown error */

  UART_STATUS_BUSY,           /**< UART Busy */

  UART_STATUS_BUFFER_INVALID, /**< Buffer provided or the buffer size is invalid*/

  UART_STATUS_MODE_MISMATCH   /**< API invoked by a handle configured with different mode.
                                   e.g, If UART_StartTransmitDMA is invoked for an instance
                                   which has transmit mode configured as "Interrupt", will
                                   return this status.*/

} UART_STATUS_t;

/**
 * @brief Enum used to describe the UART Mode of operation
 */
typedef enum UART_MODE
{
  UART_MODE_FULLDUPLEX, /**< Full Duplex mode selected */
  UART_MODE_HALFDUPLEX, /**< Half Duplex mode selected */
  UART_MODE_LOOPBACK    /**< LoopBack mode selected */
} UART_MODE_t;

/**
 * @brief Enum used to identify UART protocol event callback function
 */
typedef enum UART_EVENT
{
  UART_EVENT_SYNC_BRK,    /**< Synchronization break detected event */
  UART_EVENT_RX_NOISE,    /**< Receiver noise detected event */
  UART_EVENT_FORMAT_ERR0, /**< Frame format error at stop bit 0 event */
  UART_EVENT_FORMAT_ERR1, /**< Frame format error at stop bit 1 event */
  UART_EVENT_COLLISION,   /**< Data collision detected in half duplex mode event */
  UART_EVENT_MAX          /**< Indicates number of UART events supported*/
} UART_EVENT_t;

/**
 * @brief Enum used to identify the transfer type used for either transmit or receive function.
 */
typedef enum UART_TRANSFER_MODE
{
  UART_TRANSFER_MODE_INTERRUPT,  /**< Implement data transmit or receive using interrupts */
  UART_TRANSFER_MODE_DMA,        /**< Implement data transmit or receive using DMA */
  UART_TRANSFER_MODE_DIRECT      /**< This configuration exposes signals for external APP connection */
}UART_TRANSFER_MODE_t;

/**
 * @}
 */

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**
 * @brief Function pointer used for callback function
 */
typedef void (*UART_cbhandler)(void);
/**
 * @brief Function pointer used for initialization function
 */
typedef UART_STATUS_t (*UART_init_handler)(void);
/**
 * @ingroup UART_datastructures
 * @{
 */

/**
 * @brief Wrapper typedefinition for XMC_UART_PROTOCOL_STATUS_t.
 */
typedef XMC_UART_CH_STATUS_FLAG_t UART_PROTOCOL_STATUS_t;

/**
 * @brief Structure for transmit pin configuration.
 */
typedef struct UART_TX_CONFIG
{
  XMC_GPIO_PORT_t *const port;            /**< Pointer to the GPIO port base address */
  const uint8_t pin;                      /**< Pin number in the port*/
  const XMC_GPIO_CONFIG_t *const config;  /**< Pin configuration structure */
} UART_TX_CONFIG_t;

#if (defined(UART_TX_DMA_USED) || defined(UART_RX_DMA_USED))
/**
 * @brief Structure for DMA configuration.
 */
typedef struct UART_DMA_CONFIG
{
  const XMC_DMA_CH_CONFIG_t * dma_ch_config;   /**< Pointer to the DMA channel configuration.*/
  uint8_t dma_channel;                         /**< DMA channel number */
}UART_DMA_CONFIG_t;
#endif

/**
 * @brief Structure for holding the configuration parameters of UART channel.
 */
typedef struct UART_CONFIG
{
  const XMC_UART_CH_CONFIG_t * const channel_config;   /**< Basic UART configuration from the GUI with baud,data bits,
                                                            frame length, stop bits and parity */
#if (defined UART_TX_DMA_USED) || (defined UART_RX_DMA_USED)
  GLOBAL_DMA_t * global_dma;                           /**< Global DMA handle */
#endif
#ifdef UART_TX_DMA_USED
  const UART_DMA_CONFIG_t * const transmit_dma_config; /**< Pointer to the DMA channel configuration used for data transmission.*/
#endif
#ifdef UART_RX_DMA_USED
  const UART_DMA_CONFIG_t * const receive_dma_config;  /**< Pointer to the DMA channel configuration used for data reception.*/
#endif
  UART_init_handler fptr_uart_config;                  /**< Function pointer to configure the MUX values */
#ifdef UART_TX_INTERRUPT_USED
  UART_cbhandler tx_cbhandler;                         /**< Function pointer to hold the callback function pointer,
                                                            called when the transmission is complete */
#endif
#ifdef UART_RX_INTERRUPT_USED
  UART_cbhandler rx_cbhandler;                         /**< Function pointer to hold the callback function pointer,
                                                            called when the reception is complete */
#endif
  UART_cbhandler sync_error_cbhandler;                 /**< Function pointer to hold the callback function pointer,
                                                            called when synchronization break detected.*/
  UART_cbhandler rx_noise_error_cbhandler;             /**< Function pointer to hold the callback function pointer,
                                                            called when receiver noise is detected*/
  UART_cbhandler format_error_bit0_cbhandler;          /**< Function pointer to hold the callback function pointer,
                                                            called when format error with stop bit 0 is detected.*/
  UART_cbhandler format_error_bit1_cbhandler;          /**< Function pointer to hold the callback function pointer,
                                                            called when format error with stop bit 1 is detected.*/
  UART_cbhandler collision_error_cbhandler;            /**< Function pointer to hold the callback function pointer,
                                                            called when collision error is detected*/
  const UART_TX_CONFIG_t * tx_pin_config;              /**< Transmit pin configuration to be used during initialization
                                                            and while changing baudrate. */
  UART_MODE_t mode;                                    /**< UART operation mode */
  UART_TRANSFER_MODE_t transmit_mode;                  /**< Mode used for transmitting data. Data can be transmitted using
                                                            interrupt, DMA or direct(using polling or external APP connection.)*/
  UART_TRANSFER_MODE_t receive_mode;                   /**< Mode used for receiving data. Data can be received using
                                                            interrupt, DMA or direct(using polling or external APP connection.)*/
  XMC_USIC_CH_FIFO_SIZE_t tx_fifo_size;                /**< Transmit FIFO size configuration */
  XMC_USIC_CH_FIFO_SIZE_t rx_fifo_size;                /**< Receive FIFO size configuration */
  uint8_t tx_sr;                                       /**< Service request number assigned to transmit interrupt */
} UART_CONFIG_t;

/**
 * @brief Structure to hold the dynamic variables for the UART communication.
 */
typedef struct UART_RUNTIME
{
  uint8_t * tx_data;            /**< Pointer to the transmit data buffer*/
  uint8_t * rx_data;            /**< Pointer to the receive data buffer*/
  uint32_t tx_data_head;        /**< Tx buffer header index*/
  uint32_t tx_data_tail;        /**< Tx buffer tail index*/
  uint32_t tx_data_count;       /**< Number of bytes of data to be transmitted*/
  uint32_t tx_data_index;       /**< Index to the byte to be transmitted next in the tx_data buffer*/
  uint32_t rx_data_head;        /**< Rx buffer header index*/
  uint32_t rx_data_tail;        /**< Rx buffer tail index*/
  uint32_t rx_data_count;       /**< Number of bytes of data to be received*/
  uint32_t rx_data_index;       /**< Indicates the number of bytes currently available in the rx_data buffer*/
  volatile bool tx_busy;        /**< Status flag to indicate busy when a transmission is assigned*/
  volatile bool rx_busy;        /**< Status flag to indicate busy when a reception is assigned*/
} UART_RUNTIME_t;

/**
 * @brief Handler structure with pointers to dynamic and static parameters.
 */
typedef struct UART
{
  XMC_USIC_CH_t * const channel;              /**< USIC channel*/
  const UART_CONFIG_t * const config;         /**< UART configuration structure pointer*/
  UART_RUNTIME_t * const runtime;             /**< Pointer to the structure holding all variables,
                                                   that can change at runtime*/
} UART_t;


UART_STATUS_t UART_Init(const UART_t *const handle);
UART_STATUS_t UART_Transmit(const UART_t *const handle, uint8_t* data_ptr, uint32_t count);
UART_STATUS_t UART_StartReceiveIRQ(const UART_t *const handle, uint8_t* data_ptr, uint32_t count);
UART_STATUS_t UART_StartTransmitIRQ(const UART_t *const handle, uint8_t* data_ptr, uint32_t count);
void UART_Receive_Check(void * args);
extern UART_t UART_0;

#endif /* USER_INC_UART_H_ */
