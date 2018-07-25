// TODO: Now support 4B align read only

module flash(
    input wire clk,           	    //50MHz 时钟输入
    input wire rst,           		//复位，高有效

    //RAMOp接口
    input wire[31:0] addr,          //物理地址，低23位有效，低2位忽略
    input wire[3:0] mode,           //访存模式，详见下面定义
    input wire[31:0] wdata,         //写数据
    output wire[31:0] rdata,        //读数据
    output reg ok,                  //操作是否完成
    output wire ready,              //是否可接受请求

    //Flash存储器信号，参考 JS28F640 芯片手册
    output reg [22:0] flash_a,      //Flash地址，a0仅在8bit模式有效，16bit模式无意义
    inout wire [15:0] flash_d,      //Flash数据
    output wire flash_rp_n,         //Flash复位信号，低有效
    output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,         //Flash片选信号，低有效
    output wire flash_oe_n,         //Flash读使能信号，低有效
    output wire flash_we_n,         //Flash写使能信号，低有效
    output wire flash_byte_n        //Flash 8bit模式选择，低有效。在使用flash的16位模式时请设为1
);

// mode
parameter NOP = 4'b0000;
parameter LW  = 4'b0001;
parameter LH  = 4'b0010;
parameter LHU = 4'b0011;
parameter LB  = 4'b0100;
parameter LBU = 4'b0101;

reg [31:0] lock_addr;
reg [3:0]  lock_mode;

// Output (const)
assign flash_ce_n   = 1'b0;         // always enable flash for demo
assign flash_byte_n = 1'b1;         // 16-bit mode
assign flash_we_n   = 1'b1;         // never write
assign flash_vpen   = 1'b1;         // experience
assign flash_rp_n   = ~rst;
assign flash_d      = {16{1'bz}};   // readonly
assign flash_oe_n   = 1'b0;         // dont know why but works

reg [2:0] status;
reg [31:0] rdata_raw;
always @(posedge clk or negedge rst) begin
    if (rst == 1) begin
        rdata_raw <= 0;
        status <= 0;
        lock_addr <= 0;
        lock_mode <= 0;
        ok <= 0;
    end else begin
        rdata_raw <= rdata_raw;
        lock_addr <= lock_addr;
        lock_mode <= lock_mode;
        status <= status + 1;   // Move next by default
        ok <= 0;
        case (status)
        0: begin
            lock_addr <= addr;
            lock_mode <= mode;
            status <= 0;
            if(mode != NOP && ~mode[3]) begin   // Read: push addr, move next
                flash_a <= {addr[22:2], 2'b00};
                status <= 1;
            end else if(mode != NOP) begin      // Write: do nothing
                ok <= 1;
            end
        end
        3: begin    // Store [15:0]. Ask for [31:16]
            rdata_raw[15:0] <= flash_d; 
            flash_a <= {lock_addr[22:2], 2'b10};
        end
        6: begin    // Store [31:16]. Ok.
            rdata_raw[31:16] <= flash_d; 
            ok <= 1;
            status <= 0;
        end
        endcase
    end
end

// sign/zero extension
assign rdata = 
    (lock_mode == LH) ? {{16{rdata_raw[15]}}, rdata_raw[15:0]} :
    (lock_mode == LHU) ? {{16{1'b0}}, rdata_raw[15:0]} :
    (lock_mode == LB) ? {{24{rdata_raw[7]}}, rdata_raw[7:0]} :
    (lock_mode == LBU) ? {{24{1'b0}}, rdata_raw[7:0]} : rdata_raw;

assign ready = status == 0;

endmodule // 