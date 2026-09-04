#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console.h"
#include "metering.h"

#define CONSOLE_LINE_MAX 64

static void print_help(void)
{
    printf(
        "Commands (stand-in for the physical UI):\n"
        "  ambient           switch to the continuous ambient monitor\n"
        "  ambient <ms>      one-shot ambient exposure reading over <ms>\n"
        "  uncorded          arm un-corded flash detection\n"
        "  corded <ms>       fire trigger, integrate for <ms>, report result\n"
        "  verbose           resume the rolling ambient log\n"
        "  quiet             pause the rolling ambient log (default)\n"
        "  help              show this message\n");
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t\r\n");
    if (cmd == NULL) {
        return;
    }
    char *arg = strtok(NULL, " \t\r\n");

    if (strcmp(cmd, "ambient") == 0) {
        meter_cmd_t c = { .type = METER_CMD_AMBIENT, .arg_ms = arg ? (uint32_t)atoi(arg) : 0 };
        metering_submit_command(c);
    } else if (strcmp(cmd, "uncorded") == 0) {
        meter_cmd_t c = { .type = METER_CMD_ARM_UNCORDED, .arg_ms = 0 };
        metering_submit_command(c);
    } else if (strcmp(cmd, "corded") == 0) {
        if (arg == NULL) {
            printf("usage: corded <exposure_ms>\n");
            return;
        }
        meter_cmd_t c = { .type = METER_CMD_CORDED, .arg_ms = (uint32_t)atoi(arg) };
        metering_submit_command(c);
    } else if (strcmp(cmd, "verbose") == 0) {
        meter_cmd_t c = { .type = METER_CMD_SET_LIVE_LOG, .arg_ms = 1 };
        metering_submit_command(c);
    } else if (strcmp(cmd, "quiet") == 0) {
        meter_cmd_t c = { .type = METER_CMD_SET_LIVE_LOG, .arg_ms = 0 };
        metering_submit_command(c);
    } else if (strcmp(cmd, "help") == 0) {
        print_help();
    } else {
        printf("unknown command: %s\n", cmd);
        print_help();
    }
}

// Reads character-by-character with manual echo/backspace handling rather
// than fgets(), since the console VFS doesn't echo typed input on its own.
static void console_task(void *arg)
{
    (void)arg;
    print_help();

    char line[CONSOLE_LINE_MAX];
    size_t len = 0;

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (c == '\r' || c == '\n') {
            putchar('\n');
            line[len] = '\0';
            handle_line(line);
            len = 0;
            continue;
        }
        if ((c == '\b' || c == 127) && len > 0) {
            len--;
            printf("\b \b");
            continue;
        }
        if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
            putchar(c);
        }
    }
}

void console_init(void)
{
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
}
