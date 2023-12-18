/*
 * cli.c
 *
 *  Created on: 18 Dec 2023
 *      Author: Sicris
 */

#include "stdbool.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "BSP_uart.h"

#define CLI_TASK_PRIORITY       (1)
#define CLI_STACK_SIZE          (512)
#define CLI_MAX_INPUT_SIZE      (256)

static const char * const welcomeString = "\r\n\n"
        "******************************************************\r\n"
        "* Type 'help' to view a list of registered commands.\r\n"
        "******************************************************\r\n";
static const char * const strLineSep = "\r\n";
static const char * const strPrompt = "> ";

static bool bInit = false;
static TaskHandle_t cliTaskHandle = NULL;
static StaticTask_t cliTaskStruct;
static StackType_t cliStack[CLI_STACK_SIZE];
static char input_strBuf[CLI_MAX_INPUT_SIZE];

static void helper_blocking_send(char * pBuf, uint16_t count)
{
    uint16_t actualByteSent = 0;
    int16_t bytesToSend = count;
    uint16_t loopCnt = 0;

    while(loopCnt < 10) {
        actualByteSent = BSP_UART_send((uint8_t *)pBuf, bytesToSend);
        pBuf += actualByteSent;
        bytesToSend -= actualByteSent;
        if(bytesToSend <= 0) {
            break;
        }
        loopCnt++;
        vTaskDelay(100);
    }
}

static void cliTask(void * pvParam)
{
    int16_t inputIndex = 0;
    int16_t moreDataToFollow = 0;
    char * output_str_buf = NULL;
    char rxChar = 0;

    output_str_buf = FreeRTOS_CLIGetOutputBuffer();

    /* print welcome string */
    helper_blocking_send((char *)welcomeString, strlen(welcomeString));

    /* print prompt symbol */
    helper_blocking_send((char *)strPrompt, 2);

    while(true) {
        if(BSP_UART_receive((uint8_t *)&rxChar, 1, 10) > 0) {
            /* Accept only ASCII (0x00 - 0x7F) */
            if((rxChar < 0x00) || (rxChar > 0x7F)) {
                continue;
            }

            if(rxChar == '\n') {
                /*
                 * A newline character was received, so the input command string is
                 * complete and can be processed.  Transmit a line separator, just to
                 * make the output easier to read.
                 */
                helper_blocking_send((char *)strLineSep, strlen(strLineSep));
                helper_blocking_send((char *)strLineSep, strlen(strLineSep));

                if(strlen(input_strBuf) == 0) {
                    /* No command to process */
                    /* Just print prompt */
                    helper_blocking_send((char *)strPrompt, 2);
                    continue;
                }

                /*
                 * The command interpreter is called repeatedly until it returns
                 * pdFALSE (0).
                 */
                do {
                    /*
                     * Send the command string to the command interpreter.  Any
                     * output generated by the command interpreter will be placed in the
                     * output_str_buf buffer.
                     */
                    moreDataToFollow = FreeRTOS_CLIProcessCommand(
                                              input_strBuf,                         /* The command string.*/
                                              output_str_buf,                       /* The output buffer. */
                                              configCOMMAND_INT_MAX_OUTPUT_SIZE     /* The size of the output buffer. */
                                              );

                    /*
                     * Write the output generated by the command interpreter to the
                     * console.
                     */
                    helper_blocking_send(output_str_buf, strlen(output_str_buf));

                } while( moreDataToFollow != 0 );

                /*
                 * All the strings generated by the input command have been sent.
                 * Processing of the command is complete.  Clear the input string ready
                 * to receive the next command.
                 */
                inputIndex = 0;
                memset( input_strBuf, 0x00, sizeof(input_strBuf) );

                /* print prompt symbol */
                helper_blocking_send((char *)strPrompt, 2);
            } else {
                /*
                 * The if() clause performs the processing after a newline character
                 * is received.  This else clause performs the processing if any other
                 * character is received. */
                if( rxChar == '\r' ) {
                    /* Ignore carriage returns. */
                } else if( rxChar == '\b' ) {
                    /*
                     * Backspace was pressed.  Erase the last character in the input
                     * buffer - if there are any
                     */
                    if( inputIndex > 0 ) {
                        inputIndex--;
                        input_strBuf[ inputIndex ] = '\0';
                    }
                } else {
                    /*
                     * A character was entered.  It was not a new line, backspace
                     * or carriage return, so it is accepted as part of the input and
                     * placed into the input buffer.  When a n is entered the complete
                     * string will be passed to the command interpreter.
                     */
                    if( inputIndex < CLI_MAX_INPUT_SIZE ) {
                        input_strBuf[ inputIndex ] = rxChar;
                        inputIndex++;
                    }
                }
            }
        }
    }
}

void CLI_init(void)
{
    if(bInit != true) {
        memset(input_strBuf, 0, sizeof(input_strBuf));
        BSP_UART_init();

        cliTaskHandle = xTaskCreateStatic(cliTask,
                                          "cli",
                                          CLI_STACK_SIZE,
                                          (void *)0,
                                          CLI_TASK_PRIORITY,
                                          cliStack,
                                          &cliTaskStruct);

        bInit = true;
    }
}