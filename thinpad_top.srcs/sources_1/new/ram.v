`default_nettype none

module ram(
    input wire clk,                 //50MHz 时钟输入
    input wire rst,                 //复位，低有效

    //RAMOp接口
    input wire[31:0] addr,          //物理地址，低23位有效，低2位忽略，31位指示是否为串口
    input wire[3:0] mode,           //访存模式，详见下面定义
    input wire[31:0] wdata,         //写数据
    output wire[31:0] rdata,        //读数据
    output wire ok,                  //操作是否完成

    //CPLD串口控制器信号
    output wire uart_rdn,           //读串口信号，低有效
    output wire uart_wrn,           //写串口信号，低有效
    input wire uart_dataready,      //串口数据准备好
    input wire uart_tbre,           //发送数据标志
    input wire uart_tsre,           //数据发送完毕标志

    //BaseRAM信号
    inout wire[31:0] base_ram_data, //BaseRAM数据，低8位与CPLD串口控制器共享
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire base_ram_ce_n,       //BaseRAM片选，低有效
    output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    output wire base_ram_we_n,       //BaseRAM写使能，低有效

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr,  //ExtRAM地址
    output wire[3:0] ext_ram_be_n,   //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire ext_ram_ce_n,        //ExtRAM片选，低有效
    output wire ext_ram_oe_n,        //ExtRAM读使能，低有效
    output wire ext_ram_we_n         //ExtRAM写使能，低有效
);

// mode
parameter NOP = 4'b0000;
parameter LW  = 4'b0001;
parameter LH  = 4'b0010;
parameter LHU = 4'b0011;
parameter LB  = 4'b0100;
parameter LBU = 4'b0101;
parameter SW  = 4'b1001;
parameter SH  = 4'b1010;
parameter SB  = 4'b1100;

reg [31:0] lock_addr;
reg [3:0]  lock_mode;
reg [31:0] lock_wdata;

always @(posedge clk) begin
    lock_addr <= addr;
    lock_mode <= mode;
    lock_wdata <= wdata;
end

wire is_read = ~lock_mode[3] && (lock_mode != NOP);
wire is_write = lock_mode[3];
wire chip_selbase_n = lock_addr[22];
wire [19:0] word_sel = lock_addr[21:2];
wire [3:0] be = 
    (lock_mode == SH || lock_mode == LH || lock_mode == LHU) ? 4'b1100 :
    (lock_mode == SB || lock_mode == LB || lock_mode == LBU) ? 4'b1110 :
    /* LW, SW, NOP */ 4'b0000;

// ce
assign base_ram_ce_n = chip_selbase_n;
assign ext_ram_ce_n = ~chip_selbase_n;
// addr
assign base_ram_addr = word_sel;
assign ext_ram_addr  = word_sel;
// we
assign base_ram_we_n = ~is_write;
assign ext_ram_we_n  = ~is_write;
// oe
assign base_ram_oe_n = ~is_read;
assign ext_ram_oe_n  = ~is_read;
// be
assign base_ram_be_n = be;
assign ext_ram_be_n  = be;
// write data
assign base_ram_data = is_write? lock_wdata: {32{1'bz}};
assign ext_ram_data  = is_write? lock_wdata: {32{1'bz}};
// ok
assign ok = lock_mode != NOP;

// deals with read data
// before byte selection and extension
wire [31:0] rdata_raw = chip_selbase_n ? ext_ram_data : base_ram_data;
// sign/zero extension
assign rdata = 
    (lock_mode == LH) ? {{16{rdata_raw[15]}}, rdata_raw[15:0]} :
    (lock_mode == LHU) ? {{16{1'b0}}, rdata_raw[15:0]} :
    (lock_mode == LB) ? {{24{rdata_raw[7]}}, rdata_raw[7:0]} :
    (lock_mode == LBU) ? {{24{1'b0}}, rdata_raw[7:0]} : rdata_raw;

endmodule
