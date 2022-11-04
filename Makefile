CFLAGS = -I./build -Wall -I/Eigen -g -fPIC -std=c++11

OBJS = build/BLASReplacement.o build/Bounds.o build/Constraints.o build/Flipper.o build/Indexlist.o build/LAPACKReplacement.o build/Matrices.o build/MessageHandling.o build/Options.o build/OQPinterface.o build/QProblem.o build/QProblemB.o build/SolutionAnalysis.o build/SparseSolver.o build/SQProblem.o build/SQProblemSchur.o build/SubjectTo.o build/Utils.o build/v_repExtAMRProject.o build/v_repLib.o

OS = $(shell uname -s)
ECHO=@

ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	OPTION = -shared
	EXT = so
else
	CFLAGS += -D__APPLE__
	OPTION = -dynamiclib -current_version 1.0
	EXT = dylib
endif

TARGET = ./bin/libv_repExtAMRProject.$(EXT)

# change the following line according to the path of your V-REP main folder 
VREP_HOME = /home/paolo/Software/V-REP_PRO_EDU_V3_6_1_Ubuntu18_04

default: v_repExtAMRProjectLib
	cp ./bin/libv_repExtAMRProject.${EXT} ${VREP_HOME}

v_repExtAMRProjectLib: $(OBJS)
		@echo "Linking $(OBJS) to $(TARGET)"
		$(ECHO)$(CXX) $(CFLAGS) $(OBJS) $(OPTION) -o $(TARGET) $(LDFLAGS)

%.o: %.c
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

./build/%.o: ./%.cpp
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

clean:
		@echo "Cleaning $(OBJS) $(TARGET)"
		$(ECHO)rm -rf $(OBJS) $(TARGET)
