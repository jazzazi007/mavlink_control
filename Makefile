CC = gcc
CFALGS = -Wall -Wextra
# adapt the following falgs...
#gcc -o gnc gnc.c -lSDL2 -lm
SRCS = src/udp_connection.c
OBJS = $(SRCS:.c=.o)
NAME = gnc.out
all: $(NAME)
$(NAME): $(OBJS)
	$(CC) $(CFALGS) $(OBJS) -o $(NAME)
%.o: %.c
	$(CC) $(CFALGS) -c $< -o $@
clean:
	rm -f $(OBJS)
fclean: clean
	rm -f $(NAME)
re: fclean all
.PHONY: all clean fclean re