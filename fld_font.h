// This is more challenging to see the characters, but way more efficient in terms of code space.
const unsigned char font5x5 [] PROGMEM = {      //Numeric Font Matrix (Arranged as 5x font data + 1x kerning data)
    B00000000, B00000000, B00000000, B00000000, B00000000,  //Space (Char 0x20)
    5,// sets the space between letters 
    B01000000, B01000000, B01000000, B00000000, B01000000, 3, //!x
    B10100000, B10100000, B00000000, B00000000, B00000000, 4, //"x
    B01010000, B11111000, B01010000, B11111000, B01010000, 6, //#x
    B01111000, B10100000, B01110000, B00101000, B11110000, 6, //$x
    B10001000, B00010000, B00100000, B01000000, B10001000, 6, //%x
    B01100000, B10010000, B01100000, B10010000, B01101000, 6, //&
    B10000000, B10000000, B00000000, B00000000, B00000000, 2, //'x
    B01000000, B10000000, B10000000, B10000000, B01000000, 3, //(x
    B10000000, B01000000, B01000000, B01000000, B10000000, 3, //)x
    B10000000, B00000000, B00000000, B00000000, B00000000, 2, //*x
    B00000000, B01000000, B11100000, B01000000, B00000000, 4, //+x
    B00000000, B00000000, B00000000, B01000000, B10000000, 3, //,x
    B00000000, B00000000, B11100000, B00000000, B00000000, 4, //-x
    B00000000, B00000000, B00000000, B11000000, B11000000, 3, //.x
    B00001000, B00010000, B00100000, B01000000, B10000000, 6, ///x
    B01100000, 
    B10010000, 
    B10110000, 
    B11010000, 
    B01100000, 5, //0
    B01000000, B11000000, B01000000, B01000000, B11100000, 4, //1x
    B11100000, B00010000, B01100000, B10000000, B11110000, 5, //2x
    B11100000, B00010000, B11100000, B00010000, B11100000, 5, //3x
    B01100000, B10100000, B11110000, B00100000, B00100000, 5, //4
    B11110000, B10000000, B11100000, B00010000, B11100000, 5, //5x
    B01100000, B10000000, B11100000, B10010000, B01100000, 5, //6x
    B11110000, B00010000, B00100000, B01000000, B10000000, 5, //7x
    B01100000, B10010000, B01100000, B10010000, B01100000, 5, //8x
    B01100000, B10010000, B01110000, B00010000, B01100000, 5, //9x
    B00000000, B10000000, B00000000, B10000000, B00000000, 2, //:x
    B00000000, B01000000, B00000000, B01000000, B10000000, 3, //;x
    B00100000, B01000000, B10000000, B01000000, B00100000, 4, //<x
    B00000000, B11100000, B00000000, B11100000, B00000000, 4, //=x
    B10000000, B01000000, B00100000, B01000000, B10000000, 4, //>x
    B01100000, B00010000, B01100000, B00000000, B01000000, 5, //?x
    B01110000, 
    B10101000, 
    B11110000, 
    B10000000, 
    B01111000, 6, //@
    B01100000, B10010000, B11110000, B10010000, B10010000, 5, //Ax
    B11100000, B10010000, B11100000, B10010000, B11100000, 5, //Bx    
    B01100000, B10010000, B10000000, B10010000, B01100000, 5, //Cx
    B11100000, B10010000, B10010000, B10010000, B11100000, 5, //Dx
    B11110000, B10000000, B11100000, B10000000, B11110000, 5, //Ex
    B11110000, B10000000, B11100000, B10000000, B10000000, 5, //Fx
    B01110000, B10000000, B10110000, B10010000, B01100000, 5, //Gx
    B10010000, B10010000, B11110000, B10010000, B10010000, 5, //Hx
    B11100000, B01000000, B01000000, B01000000, B11100000, 4, //Ix
    B00010000, 
    B00010000, 
    B00010000, 
    B10010000, 
    B01100000, 5, //J
    B10010000, B10100000, B11000000, B10100000, B10010000, 5, //Kx
    B10000000, B10000000, B10000000, B10000000, B11100000, 4, //Lx
    B10001000, B11011000, B10101000, B10001000, B10001000, 6, //Mx
    B10001000, B11001000, B10101000, B10011000, B10001000, 6, //Nx
    B01100000, B10010000, B10010000, B10010000, B01100000, 5, //Ox
    B11100000, B10010000, B11100000, B10000000, B10000000, 5, //Px
    B01100000, B10010000, B10110000, B10010000, B01101000, 6, //Qx
    B11100000, B10010000, B11100000, B10100000, B10010000, 5, //Rx
    B01110000, B10000000, B01100000, B00010000, B11100000, 5, //Sx
    B11100000, B01000000, B01000000, B01000000, B01000000, 4, //Tx
    B10010000, B10010000, B10010000, B10010000, B01100000, 5, //Ux
    B10001000, B10001000, B10001000, B01010000, B00100000, 6, //Vx
    B10001000, B10001000, B10101000, B11011000, B10001000, 6, //Wx
    B10010000, B10010000, B01100000, B10010000, B10010000, 5, //Xx
    B10100000, B10100000, B01000000, B01000000, B01000000, 4, //Yx
    B11110000, B00100000, B01000000, B10000000, B11110000, 5, //Zx
    B11100000, B10000000, B10000000, B10000000, B11100000, 4, //[x
    B10000000, B01000000, B00100000, B00010000, B00001000, 6, //(Backward Slash)x
    B11100000, B00100000, B00100000, B00100000, B11100000, 4, //]x
    B00100000, B01010000, B00000000, B00000000, B00000000, 5, //^x
    B00000000, B00000000, B00000000, B00000000, B11110000, 5, //_x
    B10000000, B01000000, B00000000, B00000000, B00000000, 3, //`x
    B01100000, B00010000, B01110000, B10010000, B01100000, 5, //ax
    B10000000, B10000000, B11100000, B10010000, B11100000, 5, //bx
    B00000000, B00000000, B01100000, B10000000, B01100000, 4, //cx
    B00010000, B00010000, B01110000, B10010000, B01110000, 5, //dx
    B01100000, B10010000, B11100000, B10000000, B01110000, 6, //ex
    B01100000, B10000000, B11000000, B10000000, B10000000, 3, //fx
    B01100000, B10010000, B01110000, B00010000, B01100000, 6, //gx
    B10000000, B10000000, B11100000, B10010000, B10010000, 5, //hx
    B00000000, B10000000, B00000000, B10000000, B10000000, 2, //ix
    B01000000, B00000000, B01000000, B01000000, B10000000, 3, //jx
    B10000000, B10000000, B10100000, B11000000, B10100000, 5, //kx
    B10000000, B10000000, B10000000, B10000000, B01000000, 4, //lx
    B00000000, B00000000, B01010000, B10101000, B10101000, 6, //mx
    B00000000, B00000000, B01000000, B10100000, B10100000, 4, //nx
    B00000000, B00000000, B01100000, B10010000, B01100000, 5, //ox
    B00000000, B11100000, B10010000, B11100000, B10000000, 5, //px
    B00000000, B01110000, B10010000, B01110000, B00010000, 5, //qx
    B00000000, B10110000, B11000000, B10000000, B10000000, 5, //rx
    B01100000, B10000000, B11000000, B00100000, B11000000, 4, //sx
    B10000000, B11000000, B10000000, B10000000, B01000000, 3, //tx
    B00000000, B00000000, B10010000, B10010000, B01100000, 5, //ux
    B00000000, B00000000, B10001000, B01010000, B00100000, 6, //vx
    B00000000, B00000000, B10001000, B10101000, B01010000, 6, //wx
    B00000000, B00000000, B10100000, B01000000, B10100000, 4, //xx
    B00000000, B01010000, B01010000, B00100000, B11000000, 5, //yx
    B00000000, B11110000, B00100000, B01000000, B11110000, 5, //zx
    B01100000, B01000000, B11000000, B01000000, B01100000, 4, //{x
    B10000000, B10000000, B10000000, B10000000, B10000000, 2, //|x
    B11000000, B01000000, B01100000, B01000000, B11000000, 4, //}x
    B00000000, B00000000, B01101000, B10110000, B00000000, 6, //~x
    B00000000, B00000000, B00000000, B00000000, B00000000, 2, // (Char 0x7F)
    B00000000, B10010000, B00000000, B10010000, B01100000, 5 // smiley
};


// This is more challenging to see the characters, but way more efficient in terms of code space.
// Note this is more of a 7x5 font than 5x5, but who's counting ?  The kerning value manages it and we have 8 wide on the FLD :)
const unsigned char aurabesh5x5 [] PROGMEM = {      //Numeric Font Matrix for Aurabesh, created by TheJugg1er (Arranged as 7x font data + 1x kerning data)
    B00000000, B00000000, B00000000, B00000000, B00000000, 3, //Space (Char 0x20)
    B01000000, B10000000, B01000000, B10000000, B00000000, 3, //!
    B10100000, B11000000, B10000000, B00000000, B00000000, 4, //"
    B01010000, B11111000, B01010000, B11111000, B01010000, 6, //#
    B01010000, B11111110, B01010100, B00001000, B00010000, 8, //$ 
    B11001000, B00010000, B00100000, B01000000, B10011000, 6, //%
    B01100000, B10010000, B01100000, B10010000, B01101000, 6, //&
    B11000000, B01000000, B01000000, B00000000, B00000000, 3, //'
    B01000000, B01000000, B11000000, B01000000, B01000000, 3, //(
    B10000000, B10000000, B11000000, B10000000, B10000000, 3, //)
    B00100000, B10101000, B01110000, B10101000, B00100000, 6, //*
    B00100000, B00100000, B11111000, B00100000, B00100000, 6, //+
    B00000000, B00000000, B00000000, B10000000, B10000000, 2, //, x
    B01100000, B00000000, B00000000, B00000000, B00000000, 4, //- x
    B00000000, B00000000, B00000000, B10100000, B10100000, 4, //. x 
    B00000000, B01000000, B01000000, B10000000, B10000000, 3, /// x
    B11111000, B10001000, B10101000, B10001000, B11111000, 6, //0
    B11000000, B01000000, B01000000, B01000000, B11100000, 4, //1
    B11110000, B00010000, B11110000, B00000000, B11110000, 5, //2
    B11111000, B00001000, B11101000, B00001000, B11111000, 6, //3
    B10001000, B10001000, B11111000, B00001000, B00001000, 6, //4
    B11111000, B00000000, B11111000, B00001000, B11111000, 6, //5
    B11111000, B00000000, B11111000, B10001000, B11111000, 6, //6
    B11111000, B00001000, B00001000, B00001000, B00001000, 6, //7
    B11111000, B10001000, B11111000, B10001000, B11111000, 6, //8
    B11111000, B10001000, B11111000, B00000000, B11111000, 6, //9
    B00000000, B10000000, B01000000, B11100000, B00000000, 4, //: x
    B10000000, B10000000, B10000000, B10000000, B10000000, 2, //; x
    B00100000, B01000000, B10000000, B01000000, B00100000, 4, //< 
    B00000000, B11111000, B00000000, B11111000, B00000000, 6, //=
    B10000000, B01000000, B00100000, B01000000, B10000000, 4, //>
    B11000000, B10100000, B00100000, B00100000, B01000000, 5, //?
    B01110000, B10101000, B11110000, B10000000, B01111000, 6, //@
    B10000100, B11111000, B00000000, B11111000, B10000100, 7, //A
    B01111000, B10000100, B00110000, B10000100, B01111000, 7, //B
    B01000000, B01000000, B01010100, B00000100, B00000100, 7, //C
    B11111100, B00001000, B01110000, B00100000, B01000000, 7, //D
    B10001110, B10001100, B01010100, B01100100, B00100100, 7, //E
    B00000000, B00010010, B11111100, B10010000, B11111110, 8, //F
    B10111110, B10100010, B10000100, B10001000, B11110000, 8, //G
    B11111000, B00000000, B01110000, B00000000, B11111000, 6, //H
    B00100000, B01100000, B00100000, B00100000, B00100000, 4, //I
    B00000010, B00001100, B11111000, B00010000, B11100000, 8, //J
    B11111000, B00001000, B00001000, B00001000, B11111000, 6, //K
    B00010000, B00010000, B10010000, B01010000, B00110000, 5, //L
    B00001100, B00010000, B00100000, B01000000, B11111100, 7, //M
    B01000000, B10001000, B10010100, B10100010, B01000001, 9, //N - Just not happy!
    B00011000, B00100100, B01000010, B10000001, B01111110, 9, //O
    B01100100, B10000100, B10000100, B01000100, B00111100, 7, //P
    B01111100, B01000100, B01000000, B01000000, B01110000, 7, //Q
    B11111000, B00010000, B00100000, B01000000, B10000000, 6, //R
    B01000010, B00100010, B00010010, B11001010, B00110110, 8, //S not terrible
    B00010000, B00010000, B10010010, B01010100, B00111000, 8, //T
    B10011100, B10100100, B10000100, B10000100, B11111100, 7, //U
    B01000100, B00101000, B00010000, B00010000, B00010000, 7, //V
    B11111100, B10000100, B10000100, B11111100, B00000000, 7, //W
    B00010000, B00101000, B01000100, B00111000, B00000000, 7, //X
    B11100010, B10100010, B01000100, B00101000, B00010000, 7, //Y
    B00000100, B00011100, B00100100, B10000100, B11111100, 7, //Z
    B11100000, B10000000, B10000000, B10000000, B11100000, 4, //[x
    B10000000, B01000000, B00100000, B00010000, B00001000, 6, //(Backward Slash)x
    B11100000, B00100000, B00100000, B00100000, B11100000, 4, //]x
    B00100000, B01010000, B00000000, B00000000, B00000000, 5, //^x
    B00000000, B00000000, B00000000, B00000000, B11110000, 5, //_x
    B10000000, B01000000, B00000000, B00000000, B00000000, 3, //`x
    B10000100, B11111000, B00000000, B11111000, B10000100, 7, //a
    B01111000, B10000100, B00110000, B10000100, B01111000, 7, //b
    B01000000, B01000000, B01010100, B00000100, B00000100, 7, //c
    B11111100, B00001000, B01110000, B00100000, B01000000, 7, //d
    B10001110, B10001100, B01010100, B01100100, B00100100, 7, //e
    B00000000, B00010010, B11111100, B10010000, B11111110, 8, //f
    B10111110, B10100010, B10000100, B10001000, B11110000, 8, //g
    B11111000, B00000000, B01110000, B00000000, B11111000, 6, //h
    B00100000, B01100000, B00100000, B00100000, B00100000, 4, //i
    B00000010, B00001100, B11111000, B00010000, B11100000, 8, //j
    B11111000, B00001000, B00001000, B00001000, B11111000, 6, //k
    B00010000, B00010000, B10010000, B01010000, B00110000, 5, //l
    B00001100, B00010000, B00100000, B01000000, B11111100, 7, //m
    B01000000, B10001000, B10010100, B10100010, B01000001, 9, //n - Just not happy!
    B00011000, B00100100, B01000010, B10000001, B01111110, 9, //o
    B01100100, B10000100, B10000100, B01000100, B00111100, 7, //p
    B01111100, B01000100, B01000000, B01000000, B01110000, 7, //q
    B11111000, B00010000, B00100000, B01000000, B10000000, 6, //r
    B01000010, B00100010, B00010010, B11001010, B00110110, 8, //s not terrible
    B00010000, B00010000, B10010010, B01010100, B00111000, 8, //t
    B10011100, B10100100, B10000100, B10000100, B11111100, 7, //u
    B01000100, B00101000, B00010000, B00010000, B00010000, 7, //v
    B11111100, B10000100, B10000100, B11111100, B00000000, 7, //w
    B00010000, B00101000, B01000100, B00111000, B00000000, 7, //x
    B11100010, B10100010, B01000100, B00101000, B00010000, 7, //y
    B00000100, B00011100, B00100100, B10000100, B11111100, 7, //z
    B01100000, B01000000, B11000000, B01000000, B01100000, 4, //{x
    B10000000, B10000000, B10000000, B10000000, B10000000, 2, //|x
    B11000000, B01000000, B01100000, B01000000, B11000000, 4, //}x
    B00000000, B00000000, B01101000, B10110000, B00000000, 6, //~x
    B01100000, B10010000, B10010000, B01100000, B00000000, 5, // (Char 0x7F)
    B01100000, B01100110, B00000000, B01100110, B00011000, 5 // smiley
};
