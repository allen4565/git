module AdderUnit (a,b,cin,s,cout);

	input a,b,cin;
	output s,cout;
	wire w1,w2,w3;
	
	xor xor1 (w1,a,b);
	and and1 (w2,a,b);
	xor xor2 (s,cin,w1);
	and and2 (w3,cin,w1);
	or  or1  (cout,w2,w3);
endmodule

//-------------------------------------------------------------------------------

module Adder4 (a,b,s);
	
	input [3:0]a,b;
	output [4:0]s;
	wire [4:0]w;
	
	AdderUnit F1      ( a[0]  , b[0]  , 1'b0  , s[0]  , w[1] );
	AdderUnit F[4:2] (a[3:1],b[3:1],w[3:1],s[3:1],w[4:2]);
	
	assign s[4]=w[4];
endmodule


//-------------------------------------------------------------------------------

module Adder8 (a,b,s);
	
	input [7:0]a,b;
	output [8:0]s;
	wire [8:0]w;
	
	AdderUnit F1      ( a[0]  , b[0]  , 1'b0  , s[0]  , w[1] );
	AdderUnit F[8:2] (a[7:1],b[7:1],w[7:1],s[7:1],w[8:2]);
	
	assign s[8]=w[8];
endmodule

//-------------------------------------------------------------------------------

module Adder16 (a,b,s);
	
	input [15:0]a,b;
	output [16:0]s;
	wire [16:0]w;
	
	AdderUnit F1      ( a[0]  , b[0]  , 1'b0  , s[0]  , w[1] );
	AdderUnit F[16:2] (a[15:1],b[15:1],w[15:1],s[15:1],w[16:2]);
	
	assign s[16]=w[16];
endmodule

//------------------------------------------------------------------------------

module Adder9 (a,b,s);
	
	input [8:0]a,b;
	output [9:0]s;
	wire [9:0]w;
	
	AdderUnit A1      ( a[0]  , b[0]  , 1'b0  , s[0]  , w[1] );
	AdderUnit A[9:2] (a[8:1],b[8:1],w[8:1],s[8:1],w[9:2]);
	assign s[9]=w[9];
endmodule

//----------------------------------------------------------------------------- 

module Adder32 (a,b,s);
	
	input [31:0]a,b;
	output [32:0]s;
	wire [32:0]w;
	
	AdderUnit F1      ( a[0]  , b[0]  , 1'b0  , s[0]  , w[1] );
	AdderUnit F[32:2] (a[31:1],b[31:1],w[31:1],s[31:1],w[32:2]);
	
	assign s[32]=w[32];
endmodule

//------------------------------------------------------------------------------

module SubtractUnit (a,b,M,cin,s,cout);

	input	 a,b,M,cin;
	output s,cout;
	wire   inv;
	
	xor xor1 (inv,b,M);
	AdderUnit Adr_in_Sbr(a,inv,cin,s,cout);
endmodule

//-----------------------------------------------------------------------------

module Subtractor4 (a,b,d);

	input [3:0]a,b;
	output[4:0]d;
	wire [4:0]s,w;
	wire  [4:1]c;
	wire  [3:0]x;
	
	SubtractUnit S1  (a[0], b[0] ,1'b1,1'b1 ,s[0] ,c[1] );
	SubtractUnit S[4:2]  (a[3:1], b[3:1] ,1'b1,c[3:1] ,s[3:1] ,c[4:2] );

	assign s[4]=c[4];
	xor trans[3:0]( x[3:0], s[3:0], ~s[4]);
	Adder4 Sbr4 (x[3:0],~s[4],w[4:0]);
	assign d[4:0]={s[4], w[3:0]};
endmodule

//-----------------------------------------------------------------------------

module Subtractor8 (a,b,d);

	input [7:0]a,b;
	output[8:0]d;
	wire [8:0]s, w;
	wire  [8:1]c;
	wire  [7:0]x;
	
	SubtractUnit S1  (a[0], b[0] ,1'b1,1'b1 ,s[0] ,c[1] );
	SubtractUnit S[8:2]  (a[7:1], b[7:1] ,1'b1,c[7:1] ,s[7:1] ,c[8:2] );

	assign s[8]=c[8];
	xor trans[7:0]( x[7:0], s[7:0], ~s[8]);
	Adder8 Sbr8 (x[7:0],~s[8],w[8:0]);
	assign d[8:0]={s[8], w[7:0]};
endmodule

//-----------------------------------------------------------------------------

module Subtractor16 (a,b,d);

	input [15:0]a,b;
	output[16:0]d;
	wire [16:0]s, w;
	wire  [16:1]c;
	wire  [15:0]x, com;
	
	SubtractUnit S1  (a[0], b[0] ,1'b1,1'b1 ,s[0] ,c[1] );
	SubtractUnit S[16:2]  (a[15:1], b[15:1] ,1'b1,c[15:1] ,s[15:1] ,c[16:2] );

	assign s[16]=c[16];
	xor trans[15:0]( x[15:0], s[15:0], ~s[16]);
	assign com[15:0] = {~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16],~s[16]};
	Adder16 Sbr16 (x[15:0],com[15:0],w[16:0]);
	assign d[16:0]={s[16], w[15:0]} ;
endmodule

//------------------------------------------------------------------------------

module D_FF (clk,D,Q,Qn);

	input clk,D;
	inout Q, Qn;
	wire r,s;
	wire w1,w2,w3;
	
	nand ( w1 ,   D ,  r );
	nand ( w2 ,  w1 ,  s );
	nand (  s , clk , w2 );
	 and ( w3 , clk , w1 );
	nand (  r ,   s , w3 );
	
	nand ( Q , s , Qn );
	nand ( Qn , r , Q );
endmodule

//------------------------------------------------------------------------------

module DFF4 ( clk, D, Q, Qn );
	
	input clk;
	input [3:0] D;
	inout [3:0] Q, Qn;
	
	D_FF DFF[3:0]( clk, D[3:0], Q[3:0], Qn[3:0] );
endmodule

//------------------------------------------------------------------------------

module DFF8 ( clk, D, Q, Qn );
	
	input clk;
	input [7:0] D;
	inout [7:0] Q, Qn;
	
	D_FF DFF[7:0]( clk, D[7:0], Q[7:0], Qn[7:0] );
endmodule

//------------------------------------------------------------------------------

module DFF16 ( clk, D, Q, Qn );
	
	input clk;
	input [15:0] D;
	inout [15:0] Q, Qn;
	
	D_FF DFF[15:0]( clk, D[15:0], Q[15:0], Qn[15:0] );
endmodule

//------------------------------------------------------------------------------

module mux (a,c1,c0,q);

	input a,c1,c0;
	output q;
	wire ac1,anot,anotc0;
	
	not (anot,a);
	nand (ac1,a,c1);
	nand (anotc0,anot,c0);
	nand (q,ac1,anotc0);
endmodule

//------------------------------------------------------------------------------

module mux4 (x,c1,c0,q);

	input x;
	input [3:0]c1,c0;
	output [3:0] q;
	
	mux m4[3:0] (x, c1[3:0], c0[3:0],q[3:0]);
	
endmodule

//------------------------------------------------------------------------------

module mux8 (x,c1,c0,q);

	input x;
	input [7:0]c1,c0;
	output [7:0] q;
	
	mux m8[7:0] (x, c1[7:0], c0[7:0],q[7:0]);
	
endmodule

//------------------------------------------------------------------------------

module mux16 (x,c1,c0,q);

	input x;
	input [15:0]c1,c0;
	output [15:0] q;
	
	mux m16[15:0] (x, c1[15:0], c0[15:0],q[15:0]);
	
endmodule

//------------------------------------------------------------------------------

module DcounterUnit (clk, re, A, D);

	input clk, re;
	output A;
	inout D;
	wire w;
	
	mux reset ( re, 1'b0, D, w );
	D_FF In_Dcounter ( clk, w, A, D );

endmodule

//------------------------------------------------------------------------------

module sync (clk, clkAst, D, Q);
	
	input clk, clkAst;
	input D;
	output Q;
	wire q, t, dustbin1, dustbin2;
	
	D_FF DFF1( clk, D, q, dustbin1 );
	D_FF DFF2( clk, q, t, dustbin2 );
	mux assist ( clkAst, t, 1'b0, Q );
endmodule

//------------------------------------------------------------------------------

module DFF_XOR ( clk, D, Q );

	input clk, D;
	inout Q;
	wire w, dustbin;
	
	D_FF _XOR ( clk, D, w, dustbin );
	xor ( Q, D, w );
endmodule

//------------------------------------------------------------------------------

module XOR_DFF ( clk, D, Q );
	
	input clk, D;
	inout Q;
	wire w, dustbin;
	
	xor ( w, D, Q );
	D_FF XOR_ ( clk, w, Q, dustbin );
endmodule

//------------------------------------------------------------------------------

module SRlatch ( s, r, Q, Qn );

	input s, r ;
	inout Q, Qn;
	
	nor ( Q, r, Qn );
	nor ( Qn, s, Q );
endmodule

//------------------------------------------------------------

module MtpUnit8 (a,ap,b,an,out);
	
	input  [15:0] a, ap;
	input b;
	wire [15:0] t;
	output [15:0] an, out;
	wire [16:0] w;
	
	mux mux[15:0]( b, a[15:0], 16'b0, t[15:0] );
	Adder16 adder16 ( ap[15:0], t[15:0], w[16:0] );
	
	assign out[15:0] = w[15:0];
	assign an[15:0] = {a[14:0],1'b0};
endmodule

//------------------------------------------------------------

module MtpUnit16 (a,ap,b,an,out);
	
	input  [31:0] a, ap;
	input b;
	wire [31:0] t;
	output [31:0] an, out;
	wire [32:0] w;
	
	mux mux[31:0]( b, a[31:0], 32'b0, t[31:0] );
	Adder32 adder32 ( ap[31:0], t[31:0], w[32:0] );
	
	assign out[31:0] = w[31:0];
	assign an[31:0] = {a[30:0],1'b0};
endmodule

//-----------------------------------------------------------

module multiplier8( A, b, p );

	input [7:0] A;
	input [7:0] b;
	output [15:0] p;
	wire [15:0] dustbin;
	wire [15:0] w1, w2, w3, w4, w5, w6, w7, w8 ;
	wire [15:0] a0, a1, a2, a3, a4, a5, a6, a7 ;
	
	assign a0[15:0] = {8'b0, A[7:0]};
	
	MtpUnit8 M1 ( a0[15:0]  , 16'b0,    b[0] , a1[15:0] , w1[15:0] );
	MtpUnit8 M2 ( a1[15:0] , w1[15:0] , b[1] , a2[15:0] , w2[15:0] );
	MtpUnit8 M3 ( a2[15:0] , w2[15:0] , b[2] , a3[15:0] , w3[15:0] );
	MtpUnit8 M4 ( a3[15:0] , w3[15:0] , b[3] , a4[15:0] , w4[15:0] );
	MtpUnit8 M5 ( a4[15:0] , w4[15:0] , b[4] , a5[15:0] , w5[15:0] );
	MtpUnit8 M6 ( a5[15:0] , w5[15:0] , b[5] , a6[15:0] , w6[15:0] );
	MtpUnit8 M7 ( a6[15:0] , w6[15:0] , b[6] , a7[15:0] , w7[15:0] );
	MtpUnit8 M8 ( a7[15:0] , w7[15:0] , b[7] , dustbin[15:0] , w8[15:0] );

	assign p[15:0] = w8[15:0];	
endmodule

//-------------------------------------------------------------------------------

module multiplier16( A, b, p );

	input [15:0] A;
	input [15:0] b;
	output [31:0] p;
	wire [31:0] dustbin;
	wire [31:0] w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16 ;
	wire [31:0] a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15 ;
	
	assign a0[31:0] = {16'b0, A[15:0]};
	
	MtpUnit16 M1 ( a0[31:0]  , 32'b0,     b[0] , a1[31:0] , w1[31:0] );
	MtpUnit16 M2 ( a1[31:0] , w1[31:0] , b[1] , a2[31:0] , w2[31:0] );
	MtpUnit16 M3 ( a2[31:0] , w2[31:0] , b[2] , a3[31:0] , w3[31:0] );
	MtpUnit16 M4 ( a3[31:0] , w3[31:0] , b[3] , a4[31:0] , w4[31:0] );
	MtpUnit16 M5 ( a4[31:0] , w4[31:0] , b[4] , a5[31:0] , w5[31:0] );
	MtpUnit16 M6 ( a5[31:0] , w5[31:0] , b[5] , a6[31:0] , w6[31:0] );
	MtpUnit16 M7 ( a6[31:0] , w6[31:0] , b[6] , a7[31:0] , w7[31:0] );
	MtpUnit16 M8 ( a7[31:0] , w7[31:0] , b[7] , a8[31:0] , w8[31:0] );
	MtpUnit16 M9 ( a8[31:0] , w8[31:0] , b[8] , a9[31:0] , w9[31:0] );
	MtpUnit16 M10 (  a9[31:0] , w9[31:0] , b[9] , a10[31:0] , w10[31:0] );
	MtpUnit16 M11 ( a10[31:0] , w10[31:0] , b[10] , a11[31:0] , w11[31:0] );
	MtpUnit16 M12 ( a11[31:0] , w11[31:0] , b[11] , a12[31:0] , w12[31:0] );
	MtpUnit16 M13 ( a12[31:0] , w12[31:0] , b[12] , a13[31:0] , w13[31:0] );
	MtpUnit16 M14 ( a13[31:0] , w13[31:0] , b[13] , a14[31:0] , w14[31:0] );
	MtpUnit16 M15 ( a14[31:0] , w14[31:0] , b[14] , a15[31:0] , w15[31:0] );
	MtpUnit16 M16 ( a15[31:0] , w15[31:0] , b[15] , dustbin[31:0] , w16[31:0] );

	assign p[31:0] = w16[31:0];	
endmodule

//-------------------------------------------------------------------------------

module DivUnit ( in, ain, b, q, out );
	
	input ain;
	input [14:0] in;
	input [15:0] b;
	output q;
	output [15:0] out;
	wire [15:0] a;
	wire [16:0] d;
	
	assign a[15:0] = {in[14:0], ain};
	Subtractor16 subtractor ( a[15:0], b[15:0], d[16:0] );
	mux16 mux_out( d[16], d[15:0], a[15:0], out[15:0] );
	
	assign q = d[16];
endmodule

//-------------------------------------------------------------

module divider16 ( a, b, q, r );

	input [15:0] a, b;
	output[15:0] q, r;
	wire [15:0] w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15;

	DivUnit D1 ( 15'b0, a[15], b[15:0], q[15], w1[15:0] );
	DivUnit D2 ( w1[14:0], a[14], b[15:0], q[14], w2[15:0] );
	DivUnit D3 ( w2[14:0], a[13], b[15:0], q[13], w3[15:0] );
	DivUnit D4 ( w3[14:0], a[12], b[15:0], q[12], w4[15:0] );
	DivUnit D5 ( w4[14:0], a[11], b[15:0], q[11], w5[15:0] );
	DivUnit D6 ( w5[14:0], a[10], b[15:0], q[10], w6[15:0] );
	DivUnit D7 ( w6[14:0], a[9], b[15:0], q[9], w7[15:0] );
	DivUnit D8 ( w7[14:0], a[8], b[15:0], q[8], w8[15:0] );
	DivUnit D9 ( w8[14:0], a[7], b[15:0], q[7], w9[15:0] );
	DivUnit D10 ( w9[14:0], a[6], b[15:0], q[6], w10[15:0] );
	DivUnit D11 ( w10[14:0], a[5], b[15:0], q[5], w11[15:0] );
	DivUnit D12 ( w11[14:0], a[4], b[15:0], q[4], w12[15:0] );
	DivUnit D13 ( w12[14:0], a[3], b[15:0], q[3], w13[15:0] );
	DivUnit D14 ( w13[14:0], a[2], b[15:0], q[2], w14[15:0] );
	DivUnit D15 ( w14[14:0], a[1], b[15:0], q[1], w15[15:0] );
	DivUnit D16 ( w15[14:0], a[0], b[15:0], q[0], r[15:0] );
endmodule

//---------------------------------------------------------------------------------

module handshake ( clkA, clkB, clkAst1, clkAst2, Da, Db, ReqA, AckA );

	input clkA, clkB, clkAst1, clkAst2;//clkAst1 rise at the third posedge of clkA, clkAst2 rise at the first posedge of clkB 
	input Da, ReqA;//ReqA rise between the third negedge of clkA and the third posedge of clkB
	output Db, AckA;
	wire Req, Ack, ReqB;
	wire D, syncB_ReqB, syncA_AckA, Reqb, Acka ;
	wire dustbin;

	//Data Route
	mux enable ( ReqB, Da, Db, D );
	D_FF DFF_Db ( clkB, D, Db, dustbin );
	
	//Req Route
	XOR_DFF XDA( clkA, ReqA, Req );
	sync syncB ( clkB, clkAst1, Req, syncB_ReqB );
	DFF_XOR DXA( clkB, syncB_ReqB, Reqb );
	mux ast_ReqB ( clkAst2, Reqb, 1'b0, ReqB );
	
	//Ack Route
	XOR_DFF XDB( clkB, ReqB, Ack );
	sync syncA ( clkA, 1'b1, Ack, syncA_AckA );
	DFF_XOR DXB( clkA, syncA_AckA, Acka );
	mux ast_AckA ( clkAst1, Acka, 1'b0, AckA );
endmodule

//---------------------------------------------------------------------------------

module Decoder_2x4( EN, k, word);
	
	input EN;
	input [1:0] k ;
	output[3:0] word ;
	wire l;
	
	and ( word[0], ~k[0], ~k[1], EN );
	
	xor  ( l, k[0], k[1] );	
	and ( word[1], k[0], l, EN );
	and ( word[2], k[1], l, EN );
	
	and ( word[3], k[0], k[1], EN );
endmodule

//--------------------------------------------------------------

module BC ( select, op, Din, q, Dout );

	input select, Din, op;
	inout q;
	output Dout;
	wire s,r, dustbin; 
	
	and( s, select, ~op, Din);
	and( r, select, ~op, ~Din);
	
	SRlatch SR ( s, r, q, dustbin );
	and( Dout, select, op, q );
endmodule

//---------------------------------------------------------------------------

module datapath( Din, Dout, ad, x1, x2, clkA, clkB, Ast1, Ast2, op, EN, s, ReqA, r, D4, D9, AckA );

	input [3:0] Din;
	output [15:0] Dout;
	input [1:0] ad ;
	input [7:0] x1;
	input [15:0] x2;
	input clkA, clkB, Ast1, Ast2;
	input op, EN, s, ReqA;
	output [15:0] r;
	inout [7:0] AckA;
	wire [3:0] D1;
	wire [15:0] p;
	wire [7:0] D2, D3, D5, D6;
	wire [15:0]	q, D7, D8;
	wire [3:0] mem0, mem1, mem2, mem3, flow0, flow1, flow2, flow3, word;
	inout [7:0] D4;
	inout[15:0] D9;
	wire [7:0] dustbin1, dustbin2, dustbin3;
	
	Decoder_2x4 ( EN, ad[1:0], word[3:0] );
	
	BC BC0[3:0] ( word[0], op, Din[3:0], mem0[3:0], flow0[3:0] );
	BC BC1[3:0] ( word[1], op, Din[3:0], mem1[3:0], flow1[3:0] );
	BC BC2[3:0] ( word[2], op, Din[3:0], mem2[3:0], flow2[3:0] );
	BC BC3[3:0] ( word[3], op, Din[3:0], mem3[3:0], flow3[3:0] );
	
	or or_D1[3:0] ( D1[3:0], flow0[3:0], flow1[3:0], flow2[3:0], flow3[3:0] );
	assign D2[7:0] = {4'b0, D1[3:0]};
	
	DFF16 reg1 ( AckA, D2[7:0], D3[7:0], dustbin1[7:0] );
	handshake H[7:0]( clkA, clkB, Ast1, Ast2, D3[7:0], D4[7:0], ReqA, AckA[7:0] );
	
	mux8 MUX( s, Dout[7:0] , D4[7:0], D5[7:0] );
	DFF8 reg2 ( clkB, D5[7:0], D6[7:0], dustbin2[7:0] );
	
	multiplier8 M( D6[7:0], x1[7:0], p[15:0] );
	divider16 D( p[15:0], x2[15:0], q[15:0], r[15:0] );

	DFF8 reg3 ( clkB, q[15:0], D7[15:0], dustbin3[7:0] );
	Adder16 A( D7[15:0], D9[15:0], D8[15:0] );
	DFF8 reg4 ( clkB, D8[15:0], D9[15:0] );
	
	assign Dout[15:0] = D9[15:0] ;
endmodule