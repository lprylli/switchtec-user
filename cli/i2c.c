#include "commands.h"
#include "argconfig.h"
#include "common.h"

#include <switchtec/switchtec.h>
#include <switchtec/portable.h>
#include <switchtec/gas.h>
#include <switchtec/gas.h>

#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <errno.h>
#include <ctype.h>


#define CMD_DESC_READ "i2c read -p <port> -s <slave(7bit)> -o <off> -n <nbytes>"

static int i2c_read(int argc, char **argv)
{
	struct {
		uint8_t cmd;
		uint8_t port;
		uint16_t slave;
		//
		uint32_t offset;
		//
		uint16_t nb_bytes;
		uint8_t addr_size;
		uint8_t off_size;
	} input = {.cmd = 0, .addr_size = 0, .off_size = 1};

	struct {
		uint8_t bytes[128];
	} resp;

	static struct {
		struct switchtec_dev *dev;
		int port;
		int slave;
		int offset;
		int count;
	} cfg = { .count = 1 };
	const struct argconfig_options opts[] = {
		DEVICE_OPTION,
		{"port", 'p', "VAL", CFG_INT, &cfg.port, required_argument,
		 "TWI port to use "},
		{"slave", 's', "VAL", CFG_INT, &cfg.slave, required_argument,
		 "i2c slave address (7bit) to access "},
		{"offset", 'o', "VAL", CFG_INT, &cfg.offset, required_argument,
		 "i2c slave address (7bit) to access "},
		{"count", 'n', "NUM", CFG_INT, &cfg.count, required_argument,
		 "number of bytes to read (default is 1)"},
		{NULL}};

	argconfig_parse(argc, argv, CMD_DESC_READ, opts, &cfg, sizeof(cfg));

	input.port = cfg.port;
	input.slave = cfg.slave * 2;
	input.offset = cfg.offset;
	input.nb_bytes = cfg.count;

	if (cfg.count > 128) {
		fprintf(stderr, "max bytes = 128, requested = %d\n", cfg.count);
		exit(1);
	}
		
	int rc = switchtec_cmd(cfg.dev, MRPC_TWI, &input, sizeof(input),
		       &resp, sizeof(resp));

	if (rc) {
		switchtec_perror("i2c_read");
		return rc;
	}

	for (int i=0;i < cfg.count;i++) {
		printf("%02x ", resp.bytes[i]);
		if (i % 16 == 15)
			printf("\n");
	}
	printf("\n");
	return 0;
}


static const struct cmd commands[] = {
	{"read", i2c_read, CMD_DESC_READ},
	{}
};

static struct subcommand subcmd = {
	.name = "i2c",
	.cmds = commands,
	.desc = "I2C interface",
	.long_desc = "Read/Write i2c, slave-size = 1 byte, offset-size = 1 byte",
};

REGISTER_SUBCMD(subcmd);
