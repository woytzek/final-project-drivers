CC = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -Wextra -Werror -g -O0
LDFLAGS +=
TARGET = gpio-rw-testapp
SRCS = testapp.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

clean:
	rm -f $(TARGET) *.o
	