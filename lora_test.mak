# Make file for lora SX1272

FILES = LoRaSender.c LoRaReceiver.c lora.a
CFLAGS = -Werror -fmax-errors=2

lora_test: $(FILES)   
	gcc $(CFLAGS) LoRaSender.c lora.a -lwiringPi -otest_s
	gcc $(CFLAGS) LoRaReceiver.c lora.a -lwiringPi -otest_r


