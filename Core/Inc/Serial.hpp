/*
 * Serial.hpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman  B Eng Hons
 */
#include	"main.h"	//	is needed here
#ifndef INC_SERIAL_HPP_
#define INC_SERIAL_HPP_

enum	class SerialErrors	{INPUT_OVERRUN, OUTPUT_OVERRUN, HAL_UART, MSG_TOO_LARGE, SOME_OTHER, SER_LAST = SOME_OTHER}	;

#if 0
/**
  * @brief  UART handle Structure definition
  */
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                /*!< UART registers base address        */

  UART_InitTypeDef         Init;                     /*!< UART communication parameters      */

  UART_AdvFeatureInitTypeDef AdvancedInit;           /*!< UART Advanced Features initialization parameters */

  const uint8_t            *pTxBuffPtr;              /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                 TxXferSize;               /*!< UART Tx Transfer size              */

  __IO uint16_t            TxXferCount;              /*!< UART Tx Transfer Counter           */

  uint8_t                  *pRxBuffPtr;              /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                 RxXferSize;               /*!< UART Rx Transfer size              */

  __IO uint16_t            RxXferCount;              /*!< UART Rx Transfer Counter           */

  uint16_t                 Mask;                     /*!< UART Rx RDR register mask          */

#if defined(USART_CR1_FIFOEN)
  uint32_t                 FifoMode;                 /*!< Specifies if the FIFO mode is being used.
                                                          This parameter can be a value of @ref UARTEx_FIFO_mode. */

  uint16_t                 NbRxDataToProcess;        /*!< Number of data to process during RX ISR execution */

  uint16_t                 NbTxDataToProcess;        /*!< Number of data to process during TX ISR execution */

#endif /*USART_CR1_FIFOEN */
  __IO HAL_UART_RxTypeTypeDef ReceptionType;         /*!< Type of ongoing reception          */

  __IO HAL_UART_RxEventTypeTypeDef RxEventType;      /*!< Type of Rx Event                   */

  void (*RxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Rx IRQ handler */

  void (*TxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Tx IRQ handler */

  DMA_HandleTypeDef        *hdmatx;                  /*!< UART Tx DMA Handle parameters      */

  DMA_HandleTypeDef        *hdmarx;                  /*!< UART Rx DMA Handle parameters      */

  HAL_LockTypeDef           Lock;                    /*!< Locking object                     */

  __IO HAL_UART_StateTypeDef    gState;              /*!< UART state information related to global Handle management
                                                          and also related to Tx operations. This parameter
                                                          can be a value of @ref HAL_UART_StateTypeDef */

  __IO HAL_UART_StateTypeDef    RxState;             /*!< UART state information related to Rx operations. This
                                                          parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t                 ErrorCode;           /*!< UART Error code                    */

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Tx Half Complete Callback        */
  void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
  void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Half Complete Callback        */
  void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */
  void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Abort Complete Callback          */
  void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart); /*!< UART Abort Transmit Complete Callback */
  void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);  /*!< UART Abort Receive Complete Callback  */
  void (* WakeupCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Wakeup Callback                  */
#if defined(USART_CR1_FIFOEN)
  void (* RxFifoFullCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Fifo Full Callback            */
  void (* TxFifoEmptyCallback)(struct __UART_HandleTypeDef *huart);       /*!< UART Tx Fifo Empty Callback           */
#endif /* USART_CR1_FIFOEN */
  void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos); /*!< UART Reception Event Callback     */

  void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
#endif  /* USE_HAL_UART_REGISTER_CALLBACKS */

} UART_HandleTypeDef;
#endif

extern	 UART_HandleTypeDef	huart1;
extern	 UART_HandleTypeDef	huart2;	//	uarts used in this project

/**	class	Serial	{
 *	Handles com ports
 */
class	Serial	{
#define		RXBUFF_SIZE			100		//	Big enough to catch all chars arriving between calls to 'test_for_message'
#define		LIVE_TXBUFF_SIZE	512		//	This many chars max passed per DMA Transmit
#define		LIN_INBUFF_SIZE		120		//	Long enough for longest command line
#define		RING_OUTBUFF_SIZE	4000	//	Large as possible but not silly - suspect buffer overrun not handled !
	UART_HandleTypeDef * const m_huartn	;		//	Which hardware usart
	volatile	bool	rx_buff_empty { true };
	volatile	bool	tx_buff_empty { true };
	volatile	bool	tx_busy { false };
	char		lin_inbuff		[LIN_INBUFF_SIZE + 4] 	{0};	//	Command line, not circular buffer
	char		ring_outbuff	[RING_OUTBUFF_SIZE + 4] {0};	//	Linear, not circular, output buffer
	char		live_tx_buff	[LIVE_TXBUFF_SIZE + 4] 	{0};	//	buffer handed to DMA Transmit
	char		ch[4] {0};
	uint32_t	lin_inbuff_onptr 	{0L};
	uint32_t	ring_outbuff_onptr 	{0L};
	uint32_t	ring_outbuff_offptr {0L};
	uint32_t	rx_onptr 			{0L};
	uint32_t	tx_onptr 			{0L};
	uint32_t	rx_offptr 			{0L};
	uint32_t	tx_offptr 			{0L};
	uint32_t	serial_error 		{0L};
	uint8_t		rxbuff	[RXBUFF_SIZE + 2] {0};	//	Circular buffer, UART Rx interrupt places data here
	bool	m_read1	(char * ch);	//	reads 1 char. Returns true on success, false when no char available
  public:
	Serial	(UART_HandleTypeDef &wot_port)	;	//	:	m_huartn {&wot_port}

	char *	test_for_message	();	//	Returns buffer address on receiving '\r', presumably at end of command string, NULL otherwise
	bool	write	(const uint8_t * t, int len)	;	//	Puts all on buffer. Transmit only once per ms
	bool	write	(const char * t, int len)	;
	bool	tx_any_buffered	()	;	//	Call this every 1ms to see if sending complete and send more if there is
	void	rx_intrpt_handler_core	();
	void	start_rx	()	;	//	Call from startup. Call from constructor is too early - CHECK!! True, but why?
	void	set_tx_busy	(bool true_or_false)	;
	bool	test_error	(SerialErrors mask)	const;	//	returns true if error bits set after masking
	int		clear_error	(SerialErrors bit);	//	-1 clears all error bits, 0 clears none, INPUT_OVERRUN_ERROR clears INPUT_OVERRUN_ERROR. Returns seial_errors
	void	set_error	(SerialErrors bit); //	{	serial_error |= (1 << bit);	}
}	;


#endif /* INC_SERIAL_HPP_ */
