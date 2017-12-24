.PHONY: clean all test install

all:
	$(MAKE) -C libscpi
	$(MAKE) -C ../linux

clean:
	$(MAKE) clean -C libscpi
	$(MAKE) clean -C ../linux

test:
	$(MAKE) test -C libscpi

install:
	$(MAKE) install -C libscpi