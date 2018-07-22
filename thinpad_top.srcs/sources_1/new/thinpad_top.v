`default_nettype none

module thinpad_top(
    input wire clk_50M,           //50MHz 时钟输入
    input wire clk_11M0592,       //11.0592MHz 时钟输入

    input wire clock_btn,         //BTN5手动时钟按钮开关，带消抖电路，按下时为1
    input wire reset_btn,         //BTN6手动复位按钮开关，带消抖电路，按下时为1

    input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
    input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时为1
    output wire[15:0] leds,       //16位LED，输出时1点亮
    output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
    output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

    //CPLD串口控制器信号
    output wire uart_rdn,         //读串口信号，低有效
    output wire uart_wrn,         //写串口信号，低有效
    input wire uart_dataready,    //串口数据准备好
    input wire uart_tbre,         //发送数据标志
    input wire uart_tsre,         //数据发送完毕标志

    //BaseRAM信号
    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire base_ram_ce_n,       //BaseRAM片选，低有效
    output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    output wire base_ram_we_n,       //BaseRAM写使能，低有效

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire ext_ram_ce_n,       //ExtRAM片选，低有效
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有效
    output wire ext_ram_we_n,       //ExtRAM写使能，低有效

    //直连串口信号
    output wire txd,  //直连串口发送端
    input  wire rxd,  //直连串口接收端

    //Flash存储器信号，参考 JS28F640 芯片手册
    output wire [22:0]flash_a,      //Flash地址，a0仅在8bit模式有效，16bit模式无意义
    inout  wire [15:0]flash_d,      //Flash数据
    output wire flash_rp_n,         //Flash复位信号，低有效
    output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,         //Flash片选信号，低有效
    output wire flash_oe_n,         //Flash读使能信号，低有效
    output wire flash_we_n,         //Flash写使能信号，低有效
    output wire flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash的16位模式时请设为1

    //USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    inout  wire[7:0] sl811_d,
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    //网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout  wire[15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    //图像输出信号
    output wire[2:0] video_red,    //红色像素，3位
    output wire[2:0] video_green,  //绿色像素，3位
    output wire[1:0] video_blue,   //蓝色像素，2位
    output wire video_hsync,       //行同步（水平同步）信号
    output wire video_vsync,       //场同步（垂直同步）信号
    output wire video_clk,         //像素时钟输出
    output wire video_de           //行数据有效信号，用于区分消隐区
);


/* =========== Demo code begin =========== */

// 数码管连接关系示意图，dpy1同理
// p=dpy0[0] // ---a---
// c=dpy0[1] // |     |
// d=dpy0[2] // f     b
// e=dpy0[3] // |     |
// b=dpy0[4] // ---g---
// a=dpy0[5] // |     |
// f=dpy0[6] // e     c
// g=dpy0[7] // |     |
//           // ---d---  p

// 7段数码管译码器演示，将number用16进制显示在数码管上面
reg[7:0] number;
SEG7_LUT segL(.oSEG1(dpy0), .iDIG(number[3:0])); //dpy0是低位数码管
SEG7_LUT segH(.oSEG1(dpy1), .iDIG(number[7:4])); //dpy1是高位数码管


//直连串口接收发送演示，从直连串口收到的数据再发送出去
wire [7:0] ext_uart_rx;
reg  [7:0] ext_uart_buffer, ext_uart_tx;
wire ext_uart_ready, ext_uart_busy;
reg ext_uart_start, ext_uart_avai;

async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块，9600无检验位
    ext_uart_r(
        .clk(clk_50M),                       //外部时钟信号
        .RxD(rxd),                           //外部串行信号输入
        .RxD_data_ready(ext_uart_ready),  //数据接收到标志
        .RxD_clear(ext_uart_ready),       //清除接收标志
        .RxD_data(ext_uart_rx)             //接收到的一字节数据
    );

always @(posedge clk_50M) begin //接收到缓冲区ext_uart_buffer
    if(ext_uart_ready)begin
        ext_uart_buffer <= ext_uart_rx & 8'b11111110;
        ext_uart_avai <= 1;
    end else if(!ext_uart_busy && ext_uart_avai)begin
        ext_uart_avai <= 0;
    end
end
always @(posedge clk_50M) begin //将缓冲区ext_uart_buffer发送出去
    if(!ext_uart_busy && ext_uart_avai)begin
        ext_uart_tx <= ext_uart_buffer;
        ext_uart_start <= 1;
    end else begin
        ext_uart_start <= 0;
    end
end

async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发送模块，9600无检验位
    ext_uart_t(
        .clk(clk_50M),                  //外部时钟信号
        .TxD(txd),                      //串行信号输出
        .TxD_busy(ext_uart_busy),       //发送器忙状态指示
        .TxD_start(ext_uart_start),    //开始发送信号
        .TxD_data(ext_uart_tx)        //待发送的数据
    );

//图像输出演示，分辨率800x600@75Hz，像素时钟为50MHz
wire [11:0] hdata;
assign video_red = hdata < 266 ? 3'b111 : 0; //红色竖条
assign video_green = hdata < 532 && hdata >= 266 ? 3'b111 : 0; //绿色竖条
assign video_blue = hdata >= 532 ? 2'b11 : 0; //蓝色竖条
assign video_clk = clk_50M;
vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) vga800x600at75 (
    .clk(clk_50M),
    .hdata(hdata), //横坐标
    .vdata(),      //纵坐标
    .hsync(video_hsync),
    .vsync(video_vsync),
    .data_enable(video_de)
);
/* =========== Demo code end =========== */

reg clk25;

always @(posedge clk_50M) begin
    clk25 <= ~clk25;
end

ram ram(
    .clk(clk_50M),
    .addr(ram_addr),
    .mode(ram_mode),
    .rdata(ram_rdata_out),
    .wdata(ram_wdata),
    .ok(ram_ok),
    .base_ram_data(base_ram_data),  // Share with Serial [7:0]
    .base_ram_addr(base_ram_addr),
    .base_ram_be_n(base_ram_be_n),
    .base_ram_ce_n(base_ram_ce_n),  // Input from RAM
    .base_ram_oe_n(base_ram_oe_n),
    .base_ram_we_n(base_ram_we_n),
    .ext_ram_data(ext_ram_data),
    .ext_ram_addr(ext_ram_addr),
    .ext_ram_be_n(ext_ram_be_n),
    .ext_ram_ce_n(ext_ram_ce_n),
    .ext_ram_oe_n(ext_ram_oe_n),
    .ext_ram_we_n(ext_ram_we_n)
);

serial serial(
    .clk(clk_50M),
    .addr(uart_addr),
    .mode(uart_mode),
    .rdata(uart_rdata_out),
    .wdata(uart_wdata),
    .ok(uart_ok),
    .uart_rdn(uart_rdn),
    .uart_wrn(uart_wrn),
    .uart_dataready(uart_dataready),
    .uart_tbre(uart_tbre),
    .uart_tsre(uart_tsre),
    .base_ram_data(base_ram_data[7:0]), // Share with RAM
    .base_ram_ce_n(base_ram_ce_n)       // Output to RAM
);

reg [31:0] ram_addr,    uart_addr;
reg [ 3:0] ram_mode,    uart_mode, mode1;
reg [31:0] ram_rdata,   uart_rdata;
reg [31:0] ram_wdata,   uart_wdata;
wire       ram_ok,      uart_ok;
wire[31:0] ram_rdata_out, uart_rdata_out;

reg [15:0] led;
assign leds = led;

reg last_clock;
always @(posedge clk_50M) begin
    last_clock <= clock_btn;
end

/*  
    访存测试：
    数码管显示状态：0~6，用手动时钟按钮切换状态，时钟50MHz
    0: 输入 RAM 写入数据 sw[31:0]
    1: 输入 RAM 访存模式 sw[31:28], 地址 sw[22:0]
    2: 输入 串口 写入数据 sw[7:0], 访存模式 sw[31:28], 地址 sw[15:8]
    3: 访存执行周期，直接跳到4
    4: 获取结果周期，直接跳到5
    5: 显示 RAM 读取数据 led[14:0], 是否完成 led[15]
    6: 显示 串口 读取数据 led[14:0], 是否完成 led[15]
 */
always @(posedge clk_50M or posedge reset_btn) begin
    if(reset_btn == 1) begin
        ram_addr <= 0;
        ram_mode <= 0;
        ram_wdata <= 0;
        ram_rdata <= 0;
        uart_addr <= 0;
        uart_mode <= 0;
        uart_wdata <= 0;
        uart_rdata <= 0;
        mode1 <= 0;
        led <= 0;
        number <= 0;
    end else begin
        ram_addr <= ram_addr;
        ram_mode <= 4'b0000;
        ram_wdata <= ram_wdata;
        ram_rdata <= ram_rdata;
        uart_addr <= uart_addr;
        uart_mode <= 4'b0000;
        uart_wdata <= uart_wdata;
        uart_rdata <= uart_rdata;
        mode1 <= mode1;
        led <= led;
        number <= number;
        case(number)
            0: if(clock_btn && ~last_clock) begin
                ram_wdata <= dip_sw;
                number <= 1;
            end
            1: if(clock_btn && ~last_clock) begin
                ram_addr <= {{9{1'b0}}, dip_sw[22:0]};
                mode1 <= dip_sw[31:28];
                number <= 2;
            end
            2: if(clock_btn && ~last_clock) begin
                uart_wdata[7:0] <= dip_sw[7:0];
                uart_addr <= {{24{1'b0}}, dip_sw[15:8]};
                uart_mode <= dip_sw[31:28];
                ram_mode <= mode1;
                number <= 3;
            end
            3:  number <= 4;
            4: begin
                ram_rdata <= ram_rdata_out;
                uart_rdata <= uart_rdata_out;
                ram_rdata[15] <= ram_ok;
                uart_rdata[15] <= uart_ok;
                number <= 5;
            end
            5: if(clock_btn && ~last_clock) begin
                led <= ram_rdata;
                number <= 6;
            end
            default: if(clock_btn && ~last_clock) begin
                led <= uart_rdata;
                number <= 0;
            end
        endcase
    end
end

endmodule
