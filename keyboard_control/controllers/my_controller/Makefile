# === Controller Settings ===
TARGET = my_controller
SRC = my_controller.c

# === Compiler Settings ===
CC = gcc
CFLAGS = -Wall -O2 -fPIC
LDFLAGS = -shared

# === Webots Path ===
WEBOTS_HOME =/usr/local/webots/
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/c"
LIBS = -L"$(WEBOTS_HOME)/lib/controller" -lController

# === Output ===
all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(INCLUDE) -o $(TARGET) $(SRC) $(LIBS)

clean:
	rm -f $(TARGET)
