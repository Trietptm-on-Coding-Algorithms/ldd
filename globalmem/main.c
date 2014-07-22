#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#define BUF_SIZE 1024 
#define PATH "/dev/globalmem1"

int main()
{
	int ret;
	char wr_buf_1[BUF_SIZE] = "hello world\n";
	char wr_buf_2[BUF_SIZE] = "a b\n";
	char wr_buf_3[BUF_SIZE] = "xx\n";
	char rd_buf_1[BUF_SIZE];
	char rd_buf_2[BUF_SIZE];
	char rd_buf_3[BUF_SIZE];
	int fd;

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	printf("%s\n", PATH);
	if (fd == -1)
		perror("open fd failed\n");
	printf("fd: %d\n", fd);
	ret = write(fd, wr_buf_1, BUF_SIZE);
	printf("write ret: %d\n", ret);
	close(fd);

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	ret = read(fd, rd_buf_1, BUF_SIZE);
	printf("ret ret: %d\n", ret);
	printf("%s", rd_buf_1);
	close(fd);

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	ret = write(fd, wr_buf_2, BUF_SIZE);
	printf("write ret: %d\n", ret);
	close(fd);

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	ret = read(fd, rd_buf_2, BUF_SIZE);
	printf("read ret: %d\n", ret);
	printf("%s", rd_buf_2);
	close(fd);

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	ret = write(fd, wr_buf_3, BUF_SIZE);
	printf("write ret: %d\n", ret);
	close(fd);

	fd = open(PATH, O_RDWR | O_NONBLOCK);
	ret = read(fd, rd_buf_3, BUF_SIZE);
	printf("read ret: %d\n", ret);
	printf("%s", rd_buf_3);
	close(fd);

	return 0;
}
