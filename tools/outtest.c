#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>

static const char outbits[] =
		"..RLDUxx.....B.."
		"1.rlduxx...$!234";
static unsigned char lights[8] = { 0, 0, 0, 0x08, 0x37, 0, 0, 0 };
static unsigned char change[4];
static int fd;

static int update()
{
	int i, b;
	for (i = 0; i < 4; i++) {
		lights[i] ^= change[i];
		for (b = 0; b < 8; b++)
			if (lights[i] & (128 >> b))
				printf("%c", outbits[i * 8 + b]);
			else
				printf(" ");
	}
	printf("\n");
	for (i = 0; i < 8; i++) {
		if (i == 4)
			printf(" ");
		printf("%02x", lights[i]);
	}
	printf("\n");
	memset(change, 0, sizeof(change));
	b = write(fd, lights, 8);
	if (b < 0)
		perror("write");
}

int main(int argc, char *argv[])
{
	char c;
	char *p;
	int n;
	fd = open("/dev/piuio0", O_WRONLY);
	if (fd < 0) {
		perror("open");
		return 1;
	}
	update();
	while (read(STDIN_FILENO, &c, 1) > 0) {
		switch (c) {
		case 'q':
			return 0;
		case '\n':
			update();
			break;
		case 'R':
		case 'L':
		case 'D':
		case 'U':
		case 'r':
		case 'l':
		case 'd':
		case 'u':
		case '1':
		case '2':
		case '3':
		case '4':
		case 'B':
			p = strchr(outbits, c);
			if (!p)
				break;
			n = p - outbits;
			change[n / 8] |= 128 >> (n % 8);
		}
	}
	lights[0] = 0;
	lights[1] = 0;
	lights[2] = 0;
	lights[3] = 8;
	memset(change, 0, 4);
	update();
	close(fd);
	return 0;
}
