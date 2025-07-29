
CFLAGS += -Wall

pidxu: pidxu.c adpcm.c mongoose.c
	gcc $(CFLAGS) $^ -lrt -lm -ldl -lportaudio -lpthread -o $@

.PHONY: install

install: pidxu
	install -o root -g root ./pidxu ~pi
	install -o root -g root ./pidxu.service /etc
