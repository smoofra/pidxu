

CFLAGS += -Wall

pidxu: pidxu.c adpcm.c mongoose.c
	gcc $(CFLAGS) $^ -lrt -lm -ldl -lportaudio -lpthread -o $@
