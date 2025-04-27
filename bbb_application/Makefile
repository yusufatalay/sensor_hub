CROSS_COMPILE ?= arm-linux-gnueabihf-

CC = $(CROSS_COMPILE)gcc

TARGET = can_receiver_app

SRCS = can_receiver.c

CFLAGS = -Wall -Wextra -pedantic  -O2 -g

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)
	@echo ">>> Derleme başarılı: '$(TARGET)' oluşturuldu."

clean:
	rm -f $(TARGET) *.o core 

.PHONY: all clean
