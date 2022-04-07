// This is more challenging to see the characters, but way more efficient in terms of code space.
const unsigned char font5x5 [] PROGMEM = {      //Numeric Font Matrix (Arranged as 5x font data + 1x kerning data)
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  //Space (Char 0x20)
    5,// sets the space between letters 
    0b01000000, 0b01000000, 0b01000000, 0b00000000, 0b01000000, 3, //!x
    0b10100000, 0b10100000, 0b00000000, 0b00000000, 0b00000000, 4, //"x
    0b01010000, 0b11111000, 0b01010000, 0b11111000, 0b01010000, 6, //#x
    0b01111000, 0b10100000, 0b01110000, 0b00101000, 0b11110000, 6, //$x
    0b10001000, 0b00010000, 0b00100000, 0b01000000, 0b10001000, 6, //%x
    0b01100000, 0b10010000, 0b01100000, 0b10010000, 0b01101000, 6, //&
    0b10000000, 0b10000000, 0b00000000, 0b00000000, 0b00000000, 2, //'x
    0b01000000, 0b10000000, 0b10000000, 0b10000000, 0b01000000, 3, //(x
    0b10000000, 0b01000000, 0b01000000, 0b01000000, 0b10000000, 3, //)x
    0b10000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 2, //*x
    0b00000000, 0b01000000, 0b11100000, 0b01000000, 0b00000000, 4, //+x
    0b00000000, 0b00000000, 0b00000000, 0b01000000, 0b10000000, 3, //,x
    0b00000000, 0b00000000, 0b11100000, 0b00000000, 0b00000000, 4, //-x
    0b00000000, 0b00000000, 0b00000000, 0b11000000, 0b11000000, 3, //.x
    0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 6, ///x
    0b01100000, 
    0b10010000, 
    0b10110000, 
    0b11010000, 
    0b01100000, 5, //0
    0b01000000, 0b11000000, 0b01000000, 0b01000000, 0b11100000, 4, //1x
    0b11100000, 0b00010000, 0b01100000, 0b10000000, 0b11110000, 5, //2x
    0b11100000, 0b00010000, 0b11100000, 0b00010000, 0b11100000, 5, //3x
    0b01100000, 0b10100000, 0b11110000, 0b00100000, 0b00100000, 5, //4
    0b11110000, 0b10000000, 0b11100000, 0b00010000, 0b11100000, 5, //5x
    0b01100000, 0b10000000, 0b11100000, 0b10010000, 0b01100000, 5, //6x
    0b11110000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 5, //7x
    0b01100000, 0b10010000, 0b01100000, 0b10010000, 0b01100000, 5, //8x
    0b01100000, 0b10010000, 0b01110000, 0b00010000, 0b01100000, 5, //9x
    0b00000000, 0b10000000, 0b00000000, 0b10000000, 0b00000000, 2, //:x
    0b00000000, 0b01000000, 0b00000000, 0b01000000, 0b10000000, 3, //;x
    0b00100000, 0b01000000, 0b10000000, 0b01000000, 0b00100000, 4, //<x
    0b00000000, 0b11100000, 0b00000000, 0b11100000, 0b00000000, 4, //=x
    0b10000000, 0b01000000, 0b00100000, 0b01000000, 0b10000000, 4, //>x
    0b01100000, 0b00010000, 0b01100000, 0b00000000, 0b01000000, 5, //?x
    0b01110000, 
    0b10101000, 
    0b11110000, 
    0b10000000, 
    0b01111000, 6, //@
    0b01100000, 0b10010000, 0b11110000, 0b10010000, 0b10010000, 5, //Ax
    0b11100000, 0b10010000, 0b11100000, 0b10010000, 0b11100000, 5, //0bx    
    0b01100000, 0b10010000, 0b10000000, 0b10010000, 0b01100000, 5, //Cx
    0b11100000, 0b10010000, 0b10010000, 0b10010000, 0b11100000, 5, //Dx
    0b11110000, 0b10000000, 0b11100000, 0b10000000, 0b11110000, 5, //Ex
    0b11110000, 0b10000000, 0b11100000, 0b10000000, 0b10000000, 5, //Fx
    0b01110000, 0b10000000, 0b10110000, 0b10010000, 0b01100000, 5, //Gx
    0b10010000, 0b10010000, 0b11110000, 0b10010000, 0b10010000, 5, //Hx
    0b11100000, 0b01000000, 0b01000000, 0b01000000, 0b11100000, 4, //Ix
    0b00010000, 
    0b00010000, 
    0b00010000, 
    0b10010000, 
    0b01100000, 5, //J
    0b10010000, 0b10100000, 0b11000000, 0b10100000, 0b10010000, 5, //Kx
    0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b11100000, 4, //Lx
    0b10001000, 0b11011000, 0b10101000, 0b10001000, 0b10001000, 6, //Mx
    0b10001000, 0b11001000, 0b10101000, 0b10011000, 0b10001000, 6, //Nx
    0b01100000, 0b10010000, 0b10010000, 0b10010000, 0b01100000, 5, //Ox
    0b11100000, 0b10010000, 0b11100000, 0b10000000, 0b10000000, 5, //Px
    0b01100000, 0b10010000, 0b10110000, 0b10010000, 0b01101000, 6, //Qx
    0b11100000, 0b10010000, 0b11100000, 0b10100000, 0b10010000, 5, //Rx
    0b01110000, 0b10000000, 0b01100000, 0b00010000, 0b11100000, 5, //Sx
    0b11100000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 4, //Tx
    0b10010000, 0b10010000, 0b10010000, 0b10010000, 0b01100000, 5, //Ux
    0b10001000, 0b10001000, 0b10001000, 0b01010000, 0b00100000, 6, //Vx
    0b10001000, 0b10001000, 0b10101000, 0b11011000, 0b10001000, 6, //Wx
    0b10010000, 0b10010000, 0b01100000, 0b10010000, 0b10010000, 5, //Xx
    0b10100000, 0b10100000, 0b01000000, 0b01000000, 0b01000000, 4, //Yx
    0b11110000, 0b00100000, 0b01000000, 0b10000000, 0b11110000, 5, //Zx
    0b11100000, 0b10000000, 0b10000000, 0b10000000, 0b11100000, 4, //[x
    0b10000000, 0b01000000, 0b00100000, 0b00010000, 0b00001000, 6, //(0backward Slash)x
    0b11100000, 0b00100000, 0b00100000, 0b00100000, 0b11100000, 4, //]x
    0b00100000, 0b01010000, 0b00000000, 0b00000000, 0b00000000, 5, //^x
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11110000, 5, //_x
    0b10000000, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 3, //`x
    0b01100000, 0b00010000, 0b01110000, 0b10010000, 0b01100000, 5, //ax
    0b10000000, 0b10000000, 0b11100000, 0b10010000, 0b11100000, 5, //0bx
    0b00000000, 0b00000000, 0b01100000, 0b10000000, 0b01100000, 4, //cx
    0b00010000, 0b00010000, 0b01110000, 0b10010000, 0b01110000, 5, //dx
    0b01100000, 0b10010000, 0b11100000, 0b10000000, 0b01110000, 6, //ex
    0b01100000, 0b10000000, 0b11000000, 0b10000000, 0b10000000, 3, //fx
    0b01100000, 0b10010000, 0b01110000, 0b00010000, 0b01100000, 6, //gx
    0b10000000, 0b10000000, 0b11100000, 0b10010000, 0b10010000, 5, //hx
    0b00000000, 0b10000000, 0b00000000, 0b10000000, 0b10000000, 2, //ix
    0b01000000, 0b00000000, 0b01000000, 0b01000000, 0b10000000, 3, //jx
    0b10000000, 0b10000000, 0b10100000, 0b11000000, 0b10100000, 5, //kx
    0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b01000000, 4, //lx
    0b00000000, 0b00000000, 0b01010000, 0b10101000, 0b10101000, 6, //mx
    0b00000000, 0b00000000, 0b01000000, 0b10100000, 0b10100000, 4, //nx
    0b00000000, 0b00000000, 0b01100000, 0b10010000, 0b01100000, 5, //ox
    0b00000000, 0b11100000, 0b10010000, 0b11100000, 0b10000000, 5, //px
    0b00000000, 0b01110000, 0b10010000, 0b01110000, 0b00010000, 5, //qx
    0b00000000, 0b10110000, 0b11000000, 0b10000000, 0b10000000, 5, //rx
    0b01100000, 0b10000000, 0b11000000, 0b00100000, 0b11000000, 4, //sx
    0b10000000, 0b11000000, 0b10000000, 0b10000000, 0b01000000, 3, //tx
    0b00000000, 0b00000000, 0b10010000, 0b10010000, 0b01100000, 5, //ux
    0b00000000, 0b00000000, 0b10001000, 0b01010000, 0b00100000, 6, //vx
    0b00000000, 0b00000000, 0b10001000, 0b10101000, 0b01010000, 6, //wx
    0b00000000, 0b00000000, 0b10100000, 0b01000000, 0b10100000, 4, //xx
    0b00000000, 0b01010000, 0b01010000, 0b00100000, 0b11000000, 5, //yx
    0b00000000, 0b11110000, 0b00100000, 0b01000000, 0b11110000, 5, //zx
    0b01100000, 0b01000000, 0b11000000, 0b01000000, 0b01100000, 4, //{x
    0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 2, //|x
    0b11000000, 0b01000000, 0b01100000, 0b01000000, 0b11000000, 4, //}x
    0b00000000, 0b00000000, 0b01101000, 0b10110000, 0b00000000, 6, //~x
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 2, // (Char 0x7F)
    0b00000000, 0b10010000, 0b00000000, 0b10010000, 0b01100000, 5 // smiley
};


// This is more challenging to see the characters, but way more efficient in terms of code space.
// Note this is more of a 7x5 font than 5x5, but who's counting ?  The kerning value manages it and we have 8 wide on the FLD :)
const unsigned char aurabesh5x5 [] PROGMEM = {      //Numeric Font Matrix for Aurabesh, created by TheJugg1er (Arranged as 7x font data + 1x kerning data)
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 3, //Space (Char 0x20)
    0b01000000, 0b10000000, 0b01000000, 0b10000000, 0b00000000, 3, //!
    0b10100000, 0b11000000, 0b10000000, 0b00000000, 0b00000000, 4, //"
    0b01010000, 0b11111000, 0b01010000, 0b11111000, 0b01010000, 6, //#
    0b01010000, 0b11111110, 0b01010100, 0b00001000, 0b00010000, 8, //$ 
    0b11001000, 0b00010000, 0b00100000, 0b01000000, 0b10011000, 6, //%
    0b01100000, 0b10010000, 0b01100000, 0b10010000, 0b01101000, 6, //&
    0b11000000, 0b01000000, 0b01000000, 0b00000000, 0b00000000, 3, //'
    0b01000000, 0b01000000, 0b11000000, 0b01000000, 0b01000000, 3, //(
    0b10000000, 0b10000000, 0b11000000, 0b10000000, 0b10000000, 3, //)
    0b00100000, 0b10101000, 0b01110000, 0b10101000, 0b00100000, 6, //*
    0b00100000, 0b00100000, 0b11111000, 0b00100000, 0b00100000, 6, //+
    0b00000000, 0b00000000, 0b00000000, 0b10000000, 0b10000000, 2, //, x
    0b01100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 4, //- x
    0b00000000, 0b00000000, 0b00000000, 0b10100000, 0b10100000, 4, //. x 
    0b00000000, 0b01000000, 0b01000000, 0b10000000, 0b10000000, 3, /// x
    0b11111000, 0b10001000, 0b10101000, 0b10001000, 0b11111000, 6, //0
    0b11000000, 0b01000000, 0b01000000, 0b01000000, 0b11100000, 4, //1
    0b11110000, 0b00010000, 0b11110000, 0b00000000, 0b11110000, 5, //2
    0b11111000, 0b00001000, 0b11101000, 0b00001000, 0b11111000, 6, //3
    0b10001000, 0b10001000, 0b11111000, 0b00001000, 0b00001000, 6, //4
    0b11111000, 0b00000000, 0b11111000, 0b00001000, 0b11111000, 6, //5
    0b11111000, 0b00000000, 0b11111000, 0b10001000, 0b11111000, 6, //6
    0b11111000, 0b00001000, 0b00001000, 0b00001000, 0b00001000, 6, //7
    0b11111000, 0b10001000, 0b11111000, 0b10001000, 0b11111000, 6, //8
    0b11111000, 0b10001000, 0b11111000, 0b00000000, 0b11111000, 6, //9
    0b00000000, 0b10000000, 0b01000000, 0b11100000, 0b00000000, 4, //: x
    0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 2, //; x
    0b00100000, 0b01000000, 0b10000000, 0b01000000, 0b00100000, 4, //< 
    0b00000000, 0b11111000, 0b00000000, 0b11111000, 0b00000000, 6, //=
    0b10000000, 0b01000000, 0b00100000, 0b01000000, 0b10000000, 4, //>
    0b11000000, 0b10100000, 0b00100000, 0b00100000, 0b01000000, 5, //?
    0b01110000, 0b10101000, 0b11110000, 0b10000000, 0b01111000, 6, //@
    0b10000100, 0b11111000, 0b00000000, 0b11111000, 0b10000100, 7, //A
    0b01111000, 0b10000100, 0b00110000, 0b10000100, 0b01111000, 7, //0b
    0b01000000, 0b01000000, 0b01010100, 0b00000100, 0b00000100, 7, //C
    0b11111100, 0b00001000, 0b01110000, 0b00100000, 0b01000000, 7, //D
    0b10001110, 0b10001100, 0b01010100, 0b01100100, 0b00100100, 7, //E
    0b00000000, 0b00010010, 0b11111100, 0b10010000, 0b11111110, 8, //F
    0b10111110, 0b10100010, 0b10000100, 0b10001000, 0b11110000, 8, //G
    0b11111000, 0b00000000, 0b01110000, 0b00000000, 0b11111000, 6, //H
    0b00100000, 0b01100000, 0b00100000, 0b00100000, 0b00100000, 4, //I
    0b00000010, 0b00001100, 0b11111000, 0b00010000, 0b11100000, 8, //J
    0b11111000, 0b00001000, 0b00001000, 0b00001000, 0b11111000, 6, //K
    0b00010000, 0b00010000, 0b10010000, 0b01010000, 0b00110000, 5, //L
    0b00001100, 0b00010000, 0b00100000, 0b01000000, 0b11111100, 7, //M
    0b01000000, 0b10001000, 0b10010100, 0b10100010, 0b01000001, 9, //N - Just not happy!
    0b00011000, 0b00100100, 0b01000010, 0b10000001, 0b01111110, 9, //O
    0b01100100, 0b10000100, 0b10000100, 0b01000100, 0b00111100, 7, //P
    0b01111100, 0b01000100, 0b01000000, 0b01000000, 0b01110000, 7, //Q
    0b11111000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 6, //R
    0b01000010, 0b00100010, 0b00010010, 0b11001010, 0b00110110, 8, //S not terrible
    0b00010000, 0b00010000, 0b10010010, 0b01010100, 0b00111000, 8, //T
    0b10011100, 0b10100100, 0b10000100, 0b10000100, 0b11111100, 7, //U
    0b01000100, 0b00101000, 0b00010000, 0b00010000, 0b00010000, 7, //V
    0b11111100, 0b10000100, 0b10000100, 0b11111100, 0b00000000, 7, //W
    0b00010000, 0b00101000, 0b01000100, 0b00111000, 0b00000000, 7, //X
    0b11100010, 0b10100010, 0b01000100, 0b00101000, 0b00010000, 7, //Y
    0b00000100, 0b00011100, 0b00100100, 0b10000100, 0b11111100, 7, //Z
    0b11100000, 0b10000000, 0b10000000, 0b10000000, 0b11100000, 4, //[x
    0b10000000, 0b01000000, 0b00100000, 0b00010000, 0b00001000, 6, //(0backward Slash)x
    0b11100000, 0b00100000, 0b00100000, 0b00100000, 0b11100000, 4, //]x
    0b00100000, 0b01010000, 0b00000000, 0b00000000, 0b00000000, 5, //^x
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11110000, 5, //_x
    0b10000000, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 3, //`x
    0b10000100, 0b11111000, 0b00000000, 0b11111000, 0b10000100, 7, //a
    0b01111000, 0b10000100, 0b00110000, 0b10000100, 0b01111000, 7, //b
    0b01000000, 0b01000000, 0b01010100, 0b00000100, 0b00000100, 7, //c
    0b11111100, 0b00001000, 0b01110000, 0b00100000, 0b01000000, 7, //d
    0b10001110, 0b10001100, 0b01010100, 0b01100100, 0b00100100, 7, //e
    0b00000000, 0b00010010, 0b11111100, 0b10010000, 0b11111110, 8, //f
    0b10111110, 0b10100010, 0b10000100, 0b10001000, 0b11110000, 8, //g
    0b11111000, 0b00000000, 0b01110000, 0b00000000, 0b11111000, 6, //h
    0b00100000, 0b01100000, 0b00100000, 0b00100000, 0b00100000, 4, //i
    0b00000010, 0b00001100, 0b11111000, 0b00010000, 0b11100000, 8, //j
    0b11111000, 0b00001000, 0b00001000, 0b00001000, 0b11111000, 6, //k
    0b00010000, 0b00010000, 0b10010000, 0b01010000, 0b00110000, 5, //l
    0b00001100, 0b00010000, 0b00100000, 0b01000000, 0b11111100, 7, //m
    0b01000000, 0b10001000, 0b10010100, 0b10100010, 0b01000001, 9, //n - Just not happy!
    0b00011000, 0b00100100, 0b01000010, 0b10000001, 0b01111110, 9, //o
    0b01100100, 0b10000100, 0b10000100, 0b01000100, 0b00111100, 7, //p
    0b01111100, 0b01000100, 0b01000000, 0b01000000, 0b01110000, 7, //q
    0b11111000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 6, //r
    0b01000010, 0b00100010, 0b00010010, 0b11001010, 0b00110110, 8, //s not terrible
    0b00010000, 0b00010000, 0b10010010, 0b01010100, 0b00111000, 8, //t
    0b10011100, 0b10100100, 0b10000100, 0b10000100, 0b11111100, 7, //u
    0b01000100, 0b00101000, 0b00010000, 0b00010000, 0b00010000, 7, //v
    0b11111100, 0b10000100, 0b10000100, 0b11111100, 0b00000000, 7, //w
    0b00010000, 0b00101000, 0b01000100, 0b00111000, 0b00000000, 7, //x
    0b11100010, 0b10100010, 0b01000100, 0b00101000, 0b00010000, 7, //y
    0b00000100, 0b00011100, 0b00100100, 0b10000100, 0b11111100, 7, //z
    0b01100000, 0b01000000, 0b11000000, 0b01000000, 0b01100000, 4, //{x
    0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 2, //|x
    0b11000000, 0b01000000, 0b01100000, 0b01000000, 0b11000000, 4, //}x
    0b00000000, 0b00000000, 0b01101000, 0b10110000, 0b00000000, 6, //~x
    0b01100000, 0b10010000, 0b10010000, 0b01100000, 0b00000000, 5, // (Char 0x7F)
    0b01100000, 0b01100110, 0b00000000, 0b01100110, 0b00011000, 5 // smiley
};
