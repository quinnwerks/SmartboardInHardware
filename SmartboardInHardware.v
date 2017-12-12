/*
   Virtual Whiteboard.
	 Quinn Smith and Torin Anderson

	 Takes composite input from a digital camera. If something in the FOV of
	 the camera is blue it will output the shape of that object to the screen.

	 This circuit enables a screen with a VGA connection to function as a Whiteboard
	 The user can select from a variety of colours to draw from (Green, White, Purple).
	 Stencils can also be used to change the shape being drawn to the screen.

	 This was built for a 3 week project for a digital design course (ECE241).

	 The circuit was based on a DE2 sample project provided by the course.
	 The main component added was the pixel processing FSM's and the FSM to controls.
	 The code not writen by us is not included.
*/



/*
   Top level module
*/
module vv2(

input			CLOCK_50,				//	On Board 50 MHz


input	[3:0]	KEY,
output [9:0] LEDR,

output			VGA_CLK,   				//	VGA Clock
output			VGA_HS,					//	VGA H_SYNC
output			VGA_VS,					//	VGA V_SYNC
output			VGA_BLANK_N,				//	VGA BLANK
output			VGA_SYNC_N,				//	VGA SYNC
output	[9:0]	VGA_R,   				//	VGA Red[9:0]
output	[9:0]	VGA_G,	 				//	VGA Green[9:0]
output	[9:0]	VGA_B,   				//	VGA Blue[9:0]

input	[7:0]	TD_DATA,    			//	TV Decoder Data bus 8 bits
input			TD_HS,					//	TV Decoder H_SYNC
input			TD_VS,					//	TV Decoder V_SYNC
output			TD_RESET_N,				//	TV Decoder Reset
input TD_CLK27,

output			I2C_SCLK,
inout			I2C_SDAT,
input [9:0] SW

);

wire [4:0] red, blue;
wire [5:0] green;

wire vga_plot;
wire [8:0] vga_x;
wire [7:0] vga_y;



VGA_PLL pll(.inclk0(CLOCK_50), .c0(VGA_CLK));

Video_In vin(
	.CLOCK_50		(CLOCK_50),
	.CLOCK_27		(TD_CLK27),
	.TD_RESET		(TD_RESET_N),
	.reset			(~KEY[0]),

	.TD_DATA		(TD_DATA),
	.TD_HS			(TD_HS),
	.TD_VS			(TD_VS),

	.waitrequest	(0),

	.x				(vga_x),
	.y				(vga_y),
	.red			(red),
	.green			(green),
	.blue			(blue),
	.pixel_en		(vga_plot)
);

avconf avc(
	.I2C_SCLK		(I2C_SCLK),
	.I2C_SDAT		(I2C_SDAT),
	.CLOCK_50		(CLOCK_50),
	.reset			(~KEY[0])
);


wire [10:0] vga_ox;
wire [10:0] vga_oy;
wire [17:0] colour;
wire fsmVGAWrite;
assign LEDR[7:0] = TD_DATA;
assign LEDR[8] = fsmVGAWrite;


fullFSM pixelProcessing(
	.iX(vga_x),
	.iY(vga_y),
	.oX(vga_ox),
	.oY(vga_oy),
	.resetn(KEY[0]),
	.clock50(CLOCK_50),
	.red(red),
	.green(green),
	.blue(blue),
	.writeEnVGA(fsmVGAWrite),
	.pixel_en(vga_plot)
	);

colourLUT COLOURSEL(SW[2:0], colour);

vga_adapter VGA(
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(colour),
			.x(vga_ox),
			.y(vga_oy),
			.plot(fsmVGAWrite & SW[8]),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.clock_25(VGA_CLK));
		defparam VGA.RESOLUTION = "320x240";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 6;

endmodule

/*
   Selects the colour to be drawn.
*/
module colourLUT(
   input [2:0] penSel,
   output reg [17:0] colour
   );
   always @ (*) begin
      case (penSel)
         3'd0: colour = 18'b000000000000000000; //erase
         3'd1: colour = 18'b111111111111111111; // White
         3'd2: colour = 18'b111111000000111111; // Purple
         3'd3: colour = 18'b000000111111000000; // Green

				 default: colour = 18'd0;
      endcase
   end
endmodule

/*
  This controls which colour process FSM is doing work. Since pixels are coming
	in faster than a single fsm can handle, multiple FSM's had to be instantiated
	to process pixels in paralell.
*/
module fullFSM(
  input [8:0] iX,
  input [7:0] iY,
  output [10:0] oX, oY,
  input resetn, clock50, clock27, pixel_en,
  input [9:0] red, green, blue,
  output writeMem, writeEnVGA
  );
  wire readyA, readyB, readyC, writeMemA, writeMemB, writeMemC;
  wire [1:0] selFSM, selMem, checkStateCtrl, csA, csB, csC;
  wire goA, goB, goC;
  wire [10:0] oaX, oaY;
  fullFSMdata u0(
    .iX(iX),
    .iY(iY), .resetn(resetn), .clock50(clock50), .red(red), .green(green)
		, .blue(blue), .readyA(readyA), .readyB(readyB), .readyC(readyC),
    .writeMemA(writeMemA), .writeMemB(writeMemB), .writeMemC(writeMemC)
		, .writeEnVGA(writeEnVGA), .writeMem(writeMem),.selFSM(selFSM), .selMem(selMem)
		, .checkState()
    ,.csA(csA), .csB(csB), .csC(csC), .goA(goA), .goB(goB), .goC(goC)
		, .clock27(clock27) , .oX(oX) , .oY(oY), .oaX(oaX), .oaY(oaY), .pixel_en(pixel_en)
    );
   FSMCtrl u1(.readyA(readyA), .readyB(readyB), .readyC(readyC)
	 , .writeMemA(writeMemA), .writeMemB(writeMemB), .writeMemC(writeMemC),
   .resetn(resetn), .clock50(clock50), .selFSM(selFSM), .selMem(selMem)
	 , .checkState(checkStateCtrl), .goA(goA), .goB(goB), .goC(goC));
endmodule

/*
   This is the datapath for all pixels coming from the camera. Each pixel
	 is processed through one of the three FSM's included in this module. The go
	 signals are used to control which FSM the work is given to.
*/
module fullFSMdata(
  input [10:0] iX,
  input [10:0] iY,
  output reg [10:0] oX,
  output reg [10:0] oY,
  output [10:0] oaX, oaY,
  input resetn, clock50,
  input [9:0] red, green, blue,
  input goA, goB, goC,
  input clock27,
  output readyA, readyB, readyC,
  output writeEnVGA,
  output writeMemA, writeMemB, writeMemC,
  output writeMem,
  output [1:0] csA, csB, csC,
  input [1:0] selFSM, selMem, checkState,
  input pixel_en
  );
  wire [10:0] /*oaX,oaY, */ obX, ocX, obY, ocY;
  reg [10:0] iaX, ibX, icX, iaY, ibY, icY;
  reg [10:0] oldX, oldY;
  wire writeVGAA, writeVGAB, writeVGAC;


  //change output register to output pixel
  always @ (*) begin
      if(csA == 2'b10) begin
          oX = oaX;
          oY = oaY;
      end
      else if(csB == 2'b10) begin
          oX = obX;
          oY = obY;
      end
      else if(csB == 2'b10) begin
          oX = ocX;
          oY = ocY;
      end
      else begin
          oX = 11'b0;
          oY = 11'b0;
      end
      if(csA == 2'b11) begin
          oX = oaX;
          oY = oaY;
      end
      else if(csB== 2'b11) begin
          oX = obX;
          oY = obY;
      end
      else if(csC == 2'b11) begin
          oX = ocX;
          oY = ocY;

      end
      else begin
          oX = 11'b0;
          oY = 11'b0;
      end
  end

  colourProcessFSM A(
    .iX(iX), .iY(iY),
    .oX(oaX), .oY(oaY),
    .red(red), .green(green), .blue(blue),
    .clock50(clock50), .resetn(resetn), .ready(readyA), .go(goA),
    .writeEnVGA(writeVGAA), .writeEnSt(writeMemA), .stateCheck(csA), .pixel_en(pixel_en)
    );


  colourProcessFSM B(
    .iX(iX), .iY(iY),
    .oX(obX), .oY(obY),
    .red(red), .green(green), .blue(blue),
    .clock50(clock50), .resetn(resetn), .ready(readyB), .go(goB),
    .writeEnVGA(writeVGAB), .writeEnSt(writeMemB), .stateCheck(csB), .pixel_en(pixel_en)
  );

  colourProcessFSM C(
    .iX(iX), .iY(iY),
    .oX(ocX), .oY(ocY),
    .red(red), .green(green), .blue(blue),
    .clock50(clock50), .resetn(resetn), .ready(readyC), .go(goC),
    .writeEnVGA(writeVGAC), .writeEnSt(writeMemC), .stateCheck(csC), .pixel_en(pixel_en)
  );
  assign writeEnVGA = writeVGAA | writeVGAB | writeVGAC;
  assign writeMem = writeMemA | writeMemB | writeMemC;


endmodule

/*
   This module distributes work to the three colour process FSMs.
	 This ensures as few pixels as possible are dropped.
*/
module FSMCtrl (readyA, readyB, readyC, writeMemA, writeMemB, writeMemC,
  resetn, clock50, selFSM, selMem, checkState, goA, goB, goC);
  input readyA, readyB, readyC, writeMemA, writeMemB, writeMemC, resetn, clock50;
  output reg [1:0] selFSM, selMem;
  output reg goA, goB, goC;
  output [1:0] checkState;

  localparam              PICK_A    = 2'd0,
                          PICK_B    = 2'd1,
                          PICK_C    = 2'd2;
  reg [1:0] current_state, next_state;
  always@(*)
  begin: state_table
    case (current_state)
      PICK_A:
      begin
        if (readyA) next_state = PICK_A;
        else next_state = PICK_B;
      end
      PICK_B:
      begin
        if (readyB) next_state = PICK_B;
        else next_state = PICK_C;
      end
      PICK_C:
      begin
        if (readyC) next_state = PICK_C;
        else next_state = PICK_A;
      end
      default: next_state = PICK_A;
    endcase
  end
  assign checkState = current_state;

  always@(*)
  begin: enable_signals
    selFSM = 2'b01;
    goA = 1'b0;
    goB = 1'b0;
    goC = 1'b0;
    case (current_state)
      PICK_A:
      begin
          selFSM = 2'b01;
          goA = 1'b1;
      end
      PICK_B:
      begin
          selFSM = 2'b10;
          goB = 1'b1;
      end
      PICK_C:
      begin
          selFSM = 2'b11;
          goC = 1'b1;
      end
    endcase
  end

  always@(*)
  begin
    if (writeMemA) selMem = 2'b01;
    else if (writeMemB) selMem = 2'b10;
    else if (writeMemC) selMem = 2'b11;
    else selMem = 2'b00;
  end

  always@(posedge clock50)
  begin
    if (!resetn)
      current_state <= PICK_A;
    else
      current_state <= next_state;
  end
endmodule

/*
   Processes an individual pixel.
	 If the pixel is blue the fsm will output a signal to write to the vga.
*/
module colourProcessFSM(
  input [10:0] iX, iY,
  input [9:0] red, green, blue,
  input clock50, resetn, go,
  input pixel_en,
  output writeEnVGA, writeEnSt, ready,
  output [10:0] oX, oY,
  output [1:0] stateCheck
  );
  wire loadReg;

     colourCtrl u0(
        .red(red),
        .green(green),
        .blue(blue),
        .go(go),
        .resetn(resetn),
        .clock50(clock50),
        .bdsel(),//unused
        .loadReg(loadReg),
        .ready(ready),
        .writeEnSt(writeEnSt),
        .writeEnVGA(writeEnVGA),
        .stateCheck(stateCheck),
		  .pixel_en(pixel_en)
     );

     dataForColour u1(
       .clock50(clock50),
       .resetn(resetn),
       .ld_reg(loadReg),
       .iX(iX),
       .iY(iY),
       .oX(oX),
       .oY(oY)
       );

endmodule

/*
   This is the datapath for pixel processing. Keeps the pixel address stored
	 in memory.
*/
module dataForColour(
  input clock50,
  input resetn,
  input [10:0] iX,
  input [10:0] iY,
  input ld_reg,
  output reg [10:0] oX,
  output reg [10:0] oY
  );


  always @ (posedge clock50) begin
     if(!resetn) begin
        oX <= 11'b0;
        oY <= 11'b0;
     end
     else if (ld_reg) begin
        oX <= iX;
        oY <= iY;
     end
  end


endmodule

/*
   This controls the datapath of a single pixel processing fsm.
*/
module colourCtrl (red, green, blue, go,resetn, clock50, ready, bdsel, loadReg, writeEnSt, writeEnVGA, stateCheck, pixel_en);
  input [9:0] red, green, blue;
  input go;
  input resetn;
  input clock50;
  input pixel_en;
  output [1:0] stateCheck;
  output reg ready;
  output reg bdsel;
  output reg loadReg;
  output reg writeEnSt;
  output reg writeEnVGA;
  wire colInRange;
  assign colInRange = (blue > green & blue > red & pixel_en) ? 1'b1 : 1'b0;

  localparam        IDLE        = 2'd0,
                    LOAD_R      = 2'd1,
                    STATE_WRITE = 2'd2,
                    OUTSIG      = 2'd3;
  reg [1:0] current_state, next_case;
  assign stateCheck = current_state;
  always@(*)
  begin: state_table
    case (current_state)
      IDLE: next_case = (go & colInRange) ? LOAD_R : IDLE;
      LOAD_R: next_case = OUTSIG;
      //STATE_WRITE: next_case = OUTSIG; //for writing to non vga memory
      OUTSIG: next_case = IDLE;
      default: next_case = IDLE;
    endcase
  end

  always@(*)
  begin: enable_signals
      ready = 1'b0;
      bdsel = 1'b0;
      loadReg = 1'b0;
      writeEnSt = 1'b0;
      writeEnVGA = 1'b0;
      case (current_state)

        IDLE:
        begin
          ready = 1'b1;
        end
        LOAD_R:
        begin
          loadReg = 1'b1;
        end
        STATE_WRITE:
        begin
          writeEnSt = 1'b1;
        end
        OUTSIG:
        begin
          writeEnVGA = 1'b1;
        end
      endcase
  end

  always@(posedge clock50)
  begin: state_FFs
    if (!resetn)
      current_state <= IDLE;
    else
      current_state <= next_case;
  end
endmodule
