PASM=pasm

PASM_FLAGS=-l

PRU_BIN=pru_stick.bin

all: $(PRU_BIN)

%.bin: %.p
	$(PASM) -b $(PASM_FLAGS) $<

clean:
	rm -f $(PRU_BIN) pru_stick.lst
