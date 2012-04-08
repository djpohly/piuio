#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

static const char inbits[] = "RLCS....rldu.$..";

int main(int argc, char *argv[])
{
	int i, b;
	unsigned char buf[32];
	int fd = open("/dev/piuio0", O_RDONLY);
	if (fd < 0) {
		perror("open");
		return 1;
	}
	for (;;) {
		i = read(fd, buf, 32);
		if (i < 32) {
			perror("read");
			return 1;
		}
		for (i = 0; i < 32; i++) {
			for (b = 0; b < 8; b++) {
				if (buf[i] & (128 >> b))
					printf(".");
				else
					printf("%c", inbits[(i % 2) * 8 + b]);
			}
			if (i % 4 == 1)
				printf(" ");
			else if (i % 4 == 3) {
				printf("\n");
				i += 4;
			}
		}
		printf("\n");
		usleep(100000);
	}
	return 0;
}
