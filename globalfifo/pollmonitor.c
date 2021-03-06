#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>
#include <linux/poll.h>

#define FIFO_CLEAR 0x1
#define BUFFER_LEN 20
#define PATH "/dev/globalfifo0"

int main()
{
	int fd, num;
	char c, rd_ch[BUFFER_LEN];
	fd_set rfds, wfds;

	fd = open(PATH, O_RDONLY | O_NONBLOCK);
	if (fd != -1)
	{
		if (ioctl(fd, FIFO_CLEAR, 0) < 0)
			printf("ioctl command failed\n");

		while (1)
		{
			FD_ZERO(&rfds);
			FD_ZERO(&wfds);
			FD_SET(fd, &rfds);
			FD_SET(fd, &wfds);

			select(fd + 1, &rfds, &wfds, NULL, NULL);

			if (FD_ISSET(fd, &rfds))
				printf("Poll monitor: can be read\n");
			if (FD_ISSET(fd, &wfds))
				printf("Poll monitor: can be write\n");

			printf("continue: y/n?\n");
			scanf("%c", &c);
			getchar();
			if ( c == 'n')
				break;
		}
	}
	else
	{
		printf("Device open failed\n");
	}

	close(fd);
}
