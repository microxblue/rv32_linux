#include "Vpicorv32_wrapper.h"
#include "verilated_vcd_c.h"

double sc_time_stamp() { return 0; }

int main(int argc, char **argv, char **env)
{
	printf("Built with %s %s.\n", Verilated::productName(), Verilated::productVersion());
	printf("Recommended: Verilator 4.0 or later.\n");

	Verilated::commandArgs(argc, argv);
	Vpicorv32_wrapper* top = new Vpicorv32_wrapper;

	// Tracing (vcd)
	VerilatedVcdC* tfp = NULL;
	const char* flag_vcd = Verilated::commandArgsPlusMatch("vcd");
	if (flag_vcd && 0==strcmp(flag_vcd, "+vcd")) {
		Verilated::traceEverOn(true);
		tfp = new VerilatedVcdC;
		top->trace (tfp, 99);
		tfp->open("trace.vcd");
	}

	// Tracing (data bus, see showtrace.py)
	FILE *trace_fd = NULL;
	const char* flag_trace = Verilated::commandArgsPlusMatch("trace");
	if (flag_trace && 0==strcmp(flag_trace, "+trace")) {
		trace_fd = fopen("trace.txt", "w");
	}

	top->clk = 0;

	int      t           = 0;
	uint32_t last_pc     = 0;
	int      start_trace = 0;

	while (!Verilated::gotFinish()) {
		if (t > 200)
			top->resetn = 1;
		top->clk = !top->clk;
		top->eval();

		if (tfp) tfp->dump (t);

		if (trace_fd && top->clk ) {
			if (top->dbg_data == 0x80000000) {
				start_trace = 1;
			}
			if (start_trace && (top->dbg_data >= 0x80000000)) {
				if (last_pc != top->dbg_data) {
					fprintf(trace_fd, "%08x\n", top->dbg_data);
					last_pc = top->dbg_data;
				}
			}
		}

		t += 5;
	}

	if (tfp) tfp->close();
	delete top;
	exit(0);
}

