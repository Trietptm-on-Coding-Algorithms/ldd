#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define BUF_SIZE 1024
#define PATH "/dev/globalfifo0"

int main(int argc, char *argv[])
{
	int fd, ret;
	char wr_buf[BUF_SIZE];

	if (argc != 2)
	{
		printf("usage: %s wr_str\n", argv[0]);
		return -1;
	}

	//printf("wr_str: %s\n", argv[1]);
	strcpy(wr_buf, argv[1]);
	//printf("wr_buf: %s\n", wr_buf);
	
	fd = open(PATH, O_RDWR);
	//printf("fd: %d\n", fd);
	ret = write(fd, wr_buf, BUF_SIZE);
	//printf("ret: %d\n", ret);

	close(fd);

	return 0;
}
