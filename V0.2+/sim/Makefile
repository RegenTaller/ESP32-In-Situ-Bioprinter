# Сборка и запуск симуляции прошивки v0.2 на ПК: make run
FW := ../ESP32_InSituBioprinter_V0_2
SRC := sim.cpp $(FW)/Settings.cpp $(FW)/Control.cpp $(FW)/HeadAxis.cpp $(FW)/Dispenser.cpp $(FW)/Commands.cpp
CXXFLAGS := -std=gnu++17 -O2 -Wall -Wextra -Wno-unused-parameter -Istubs -I$(FW)

sim: $(SRC) $(wildcard $(FW)/*.h) $(wildcard stubs/*.h stubs/*/*.h)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@

run: sim
	./sim

clean:
	rm -f sim

.PHONY: run clean
