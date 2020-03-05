package MemoryLatency;

import FIFO :: *;
import SpecialFIFOs :: *;
import GetPut :: *;
import Clocks :: *;
import ClientServer :: *;
import Connectable :: *;
import DefaultValue :: *;
import Vector :: *;
import BUtils :: *;
import DReg :: *;

import BlueAXI :: *;
import BlueLib :: *;

// Configuration Interface
typedef 12 CONFIG_ADDR_WIDTH;
typedef 64 CONFIG_DATA_WIDTH;

// On FPGA Master
typedef 1 FPGA_AXI_ID_WIDTH;
typedef 64 FPGA_AXI_ADDR_WIDTH;
typedef 512 FPGA_AXI_DATA_WIDTH;
typedef TLog#(FPGA_AXI_DATA_WIDTH) FPGA_AXI_DATA_WIDTH_BITS;
typedef 0 FPGA_AXI_USER_WIDTH;

interface MemoryLatency;
    (*prefix="S_AXI"*) interface AXI4_Lite_Slave_Rd_Fab#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_rd;
    (*prefix="S_AXI"*) interface AXI4_Lite_Slave_Wr_Fab#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_wr;

    (*prefix="M_AXI"*) interface AXI4_Master_Rd_Fab#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) rd;
    (*prefix="M_AXI"*) interface AXI4_Master_Wr_Fab#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) wr;

    (* always_ready *) method Bool interrupt();
endinterface

module mkMemoryLatency#(parameter Bit#(CONFIG_DATA_WIDTH) base_address)(MemoryLatency);

    Integer reg_start = 'h00;
    Integer reg_ret = 'h10;
    Integer reg_address = 'h20;
    Integer reg_len   = 'h30;

    Reg#(Bool) start <- mkReg(False);
    Reg#(Bool) idle <- mkReg(True);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) status <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) address <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) length <- mkReg(0);

    Wire#(Bool) interrupt_w <- mkDWire(False);

    List#(RegisterOperator#(axiAddrWidth, CONFIG_DATA_WIDTH)) operators = Nil;
    operators = registerHandler(reg_start, start, operators);
    operators = registerHandler(reg_ret, status, operators);
    operators = registerHandler(reg_address, address, operators);
    operators = registerHandler(reg_len, length, operators);
    GenericAxi4LiteSlave#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_config <- mkGenericAxi4LiteSlave(operators, 1, 1);

    AXI4_Master_Rd#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) rdMaster <- mkAXI4_Master_Rd(1, 1, False);
    AXI4_Master_Wr#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) wrMaster <- mkAXI4_Master_Wr(1, 1, 1, False);

    Reg#(Bit#(CONFIG_DATA_WIDTH)) cycleCount <- mkRegU;
    Reg#(Bit#(CONFIG_DATA_WIDTH)) latency <- mkRegU;
    Reg#(Bool) first <- mkRegU;

    rule startRead if(idle && start);
        start <= False;
        idle <= False;
        cycleCount <= 0;
        first <= True;
        let burst_length = ((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS);
        rdMaster.request.put(AXI4_Read_Rq {id: 0, addr: base_address + address, burst_length: unpack(truncate(burst_length)),
            burst_size: bitsToBurstSize(valueOf(FPGA_AXI_DATA_WIDTH)), burst_type: INCR, lock: NORMAL, cache: NORMAL_NON_CACHEABLE_BUFFERABLE,
            prot: UNPRIV_SECURE_DATA, qos: 0, region: 0, user: 0});
    endrule

    Reg#(Bool) interruptR <- mkDReg(False);

    rule dropReads;
        let r <- rdMaster.response.get();
        if (first) begin
            first <= False;
            latency <= cycleCount;
        end
        if (r.last) begin
            idle <= True;
            status <= pack(latency);
            interruptR <= True;
        end
    endrule

    rule count if(!idle);
        cycleCount <= cycleCount + 1;
    endrule

    interface s_rd = s_config.s_rd;
    interface s_wr = s_config.s_wr;

    interface rd = rdMaster.fab;
    interface wr = wrMaster.fab;

    method Bool interrupt = interruptR;
endmodule

endpackage
