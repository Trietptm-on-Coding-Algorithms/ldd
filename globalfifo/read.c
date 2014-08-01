#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BUF_SIZE 1024
#define PATH "/dev/globalfifo0"

int main(int argc, char *argv[])
{
	int ret, fd;
	char rd_buf[BUF_SIZE];

	//usage: ./rd file_path rd_mode
	if (argc != 2)
	{
		printf("usage: %s rd_mode\n", argv[0]);
		printf("rd_mode: 0 block\n");
		printf("         1 non_block\n");
		return -1;
	}

	if ((atoi(argv[1])) == 0)
		fd = open(PATH, O_RDWR);
	else if ((atoi(argv[1])) == 1)
		fd = open(PATH, O_RDWR | O_NONBLOCK);
	else
		return -1;
	printf("fd: %d\n", fd);

	ret = read(fd, rd_buf, BUF_SIZE);

	printf("ret: %d\n", ret);
	if (ret == -1)
		perror("read error\n");
	printf("\n%s\n", rd_buf);

	close(fd);

	return 0;
}
