# A2091 co-simulation (Verilator + the real HPS code)
all: run

obj_dir/Va2091: harness.cpp ../rtl/a2091.v ../main_mister/minimig_a2091.cpp
	cp ../main_mister/minimig_a2091.cpp ../main_mister/minimig_a2091.h stub/support/minimig/
	verilator --cc --exe --build -j 4 -Wno-fatal -Wno-WIDTH -Wno-UNUSED \
		-CFLAGS "-I$(CURDIR)/stub -I$(CURDIR)/stub/support/minimig -O1 -g -DA2091_IDLE_US=0 -DSPI_GAP=2" \
		../rtl/a2091.v harness.cpp $(CURDIR)/stub/support/minimig/minimig_a2091.cpp -o Va2091

run: obj_dir/Va2091
	./obj_dir/Va2091

clean:
	rm -rf obj_dir sim_test.hdf a2091.rom
