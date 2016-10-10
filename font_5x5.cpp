#include <Arduino.h>
static constexpr unsigned char operator "" _b (const char *p, size_t) {
	return
		((p[0]!=' ') << 4) + 
		((p[1]!=' ') << 3) + 
		((p[2]!=' ') << 2) + 
		((p[3]!=' ') << 1) + 
		((p[4]!=' ') << 0) ; 
	}
const PROGMEM unsigned char font_5x5[][5] = {
{ // 0x21 !
"  @  "_b,
"  @  "_b,
"  @  "_b,
"     "_b,
"  @  "_b,
},
{ // 0x22 "
" @ @ "_b,
" @ @ "_b,
"     "_b,
"     "_b,
"     "_b,
},
{ // 0x23 #
" @ @ "_b,
"@@@@@"_b,
" @ @ "_b,
"@@@@@"_b,
" @ @ "_b,
},
{ // 0x24 #
" @@@ "_b,
"@ @  "_b,
" @@@ "_b,
"  @ @"_b,
" @@@ "_b,
},
{ // 0x25 %
"@@  @"_b,
"@@ @ "_b,
"  @  "_b,
" @ @@"_b,
"@  @@"_b,
},
{ // 0x26 &
" @   "_b,
"@ @  "_b,
" @ @@"_b,
"@  @ "_b,
" @@ @"_b,
},
{ // 0x27 '
"  @  "_b,
"  @  "_b,
"     "_b,
"     "_b,
"     "_b,
},
{ // 0x28 (
"  @  "_b,
" @   "_b,
" @   "_b,
" @   "_b,
"  @  "_b,
},
{ // 0x29 )
"  @  "_b,
"   @ "_b,
"   @ "_b,
"   @ "_b,
"  @  "_b,
},
{ // 0x2a *
"@ @ @"_b,
" @@@ "_b,
"@@@@@"_b,
" @@@ "_b,
"@ @ @"_b,
},
{ // 0x2b +
"  @  "_b,
"  @  "_b,
"@@@@@"_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x2c ,
"     "_b,
"     "_b,
" @@  "_b,
" @@  "_b,
"@    "_b,
},
{ // 0x2d -
"     "_b,
"     "_b,
"@@@@@"_b,
"     "_b,
"     "_b,
},
{ // 0x2e .
"     "_b,
"     "_b,
"     "_b,
"@@   "_b,
"@@   "_b,
},
{ // 0x2f /
"    @"_b,
"   @ "_b,
"  @  "_b,
" @   "_b,
"@    "_b,
},


{ // 0x30 0
" @@@ "_b,
"@  @@"_b,
"@ @ @"_b,
"@@  @"_b,
" @@@ "_b,
},
{ // 0x31 1
"  @  "_b,
" @@  "_b,
"  @  "_b,
"  @  "_b,
"@@@@@"_b,
},
{ // 0x32 2
" @@@ "_b,
"@   @"_b,
"  @@ "_b,
" @   "_b,
"@@@@@"_b,
},
{ // 0x33 3
"@@@@ "_b,
"    @"_b,
"  @@ "_b,
"    @"_b,
"@@@@ "_b,
},
{ // 0x34 4
"  @@ "_b,
" @ @ "_b,
"@@@@@"_b,
"   @ "_b,
"   @ "_b,
},
{ // 0x35 5
"@@@@@"_b,
"@    "_b,
"@@@@ "_b,
"    @"_b,
"@@@@ "_b,
},
{ // 0x36 6
" @@@@"_b,
"@    "_b,
"@@@@ "_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x37 7
"@@@@@"_b,
"@   @"_b,
"   @ "_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x38 8
" @@@ "_b,
"@   @"_b,
" @@@ "_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x39 9
" @@@ "_b,
"@   @"_b,
" @@@@"_b,
"    @"_b,
"@@@@ "_b,
},
{ // 0x3a :
"     "_b,
"  @  "_b,
"     "_b,
"  @  "_b,
"     "_b,
},
{ // 0x3b ;
"     "_b,
"  @  "_b,
"     "_b,
"  @  "_b,
" @   "_b,
},
{ // 0x3c ;
"    @"_b,
"   @ "_b,
"  @  "_b,
"   @ "_b,
"    @"_b,
},
{ // 0x3d ;
"     "_b,
"@@@@@"_b,
"     "_b,
"@@@@@"_b,
"     "_b,
},
{ // 0x3f ?
" @@@ "_b,
"@   @"_b,
"   @ "_b,
"     "_b,
"  @  "_b,
},
{ // 0x40 @
" @@@ "_b,
"@   @"_b,
"@ @@@"_b,
"@ @ @"_b,
"  @@@"_b,
},
{ // 0x41 A
" @@@ "_b,
"@   @"_b,
"@@@@@"_b,
"@   @"_b,
"@   @"_b,
},
{ // 0x42 B
"@@@@ "_b,
"@   @"_b,
"@@@@ "_b,
"@   @"_b,
"@@@@ "_b,
},
{ // 0x43 C
" @@@ "_b,
"@   @"_b,
"@    "_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x44 D
"@@@@ "_b,
"@   @"_b,
"@   @"_b,
"@   @"_b,
"@@@@ "_b,
},
{ // 0x45 E
"@@@@@"_b,
"@    "_b,
"@@@@ "_b,
"@    "_b,
"@@@@@"_b,
},
{ // 0x46 F
"@@@@@"_b,
"@    "_b,
"@@@@ "_b,
"@    "_b,
"@    "_b,
},
{ // 0x47 G
" @@@ "_b,
"@    "_b,
"@ @@@"_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x48 H
"@   @"_b,
"@   @"_b,
"@@@@@"_b,
"@   @"_b,
"@   @"_b,
},
{ // 0x49 I
"@@@@@"_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"@@@@@"_b,
},
{ // 0x4a J
"@@@@@"_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"@@   "_b,
},
{ // 0x4b K
"@   @"_b,
"@  @ "_b,
"@@@  "_b,
"@  @ "_b,
"@   @"_b,
},
{ // 0x4c L
"@    "_b,
"@    "_b,
"@    "_b,
"@    "_b,
"@@@@@"_b,
},
{ // 0x4d M
"@   @"_b,
"@@ @@"_b,
"@ @ @"_b,
"@   @"_b,
"@   @"_b,
},
{ // 0x4e N
"@   @"_b,
"@@  @"_b,
"@ @ @"_b,
"@  @@"_b,
"@   @"_b,
},
{ // 0x4f O
" @@@ "_b,
"@   @"_b,
"@   @"_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x50 P
"@@@@ "_b,
"@   @"_b,
"@@@@ "_b,
"@    "_b,
"@    "_b,
},
{ // 0x51 Q
" @@@ "_b,
"@   @"_b,
"@ @ @"_b,
"@  @@"_b,
" @@@@"_b,
},
{ // 0x52 R
"@@@@ "_b,
"@   @"_b,
"@@@@ "_b,
"@  @ "_b,
"@   @"_b,
},
{ // 0x53 S
" @@@@"_b,
"@    "_b,
" @@@ "_b,
"    @"_b,
"@@@@ "_b,
},
{ // 0x54 T
"@@@@@"_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x55 U
"@   @"_b,
"@   @"_b,
"@   @"_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x56 V
"@   @"_b,
"@   @"_b,
"@   @"_b,
" @ @ "_b,
"  @  "_b,
},
{ // 0x57 W
"@   @"_b,
"@   @"_b,
"@ @ @"_b,
"@@ @@"_b,
"@   @"_b,
},
{ // 0x58 X
"@   @"_b,
" @ @ "_b,
"  @  "_b,
" @ @ "_b,
"@   @"_b,
},
{ // 0x59 Y
"@   @"_b,
" @ @ "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x5a Z
"@@@@@"_b,
"   @ "_b,
"  @  "_b,
" @   "_b,
"@@@@@"_b,
},
{ // 0x5b [
"  @@@"_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @@@"_b,
},
{ // 0x5c '\'
"@    "_b,
" @   "_b,
"  @  "_b,
"   @ "_b,
"    @"_b,
},
{ // 0x5d ]
"@@@  "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"@@@  "_b,
},
{ // 0x5e ^
"  @  "_b,
" @ @ "_b,
"@   @"_b,
"     "_b,
"     "_b,
},
{ // 0x5f _
"     "_b,
"     "_b,
"     "_b,
"     "_b,
"@@@@@"_b,
},
{ // 0x60 `
" @   "_b,
"  @  "_b,
"     "_b,
"     "_b,
"     "_b,
},



{ // 0x61 a
" @@@ "_b,
"    @"_b,
" @@@@"_b,
"@   @"_b,
" @@@@"_b,
},
{ // 0x62 b
"@    "_b,
"@    "_b,
"@@@@ "_b,
"@   @"_b,
"@@@@ "_b,
},
{ // 0x63 c
"     "_b,
"     "_b,
" @@@@"_b,
"@    "_b,
" @@@@"_b,
},
{ // 0x64 d
"    @"_b,
"    @"_b,
" @@@@"_b,
"@   @"_b,
" @@@@"_b,
},
{ // 0x65 e
" @@@ "_b,
"@   @"_b,
"@@@@ "_b,
"@    "_b,
" @@@ "_b,
},
{ // 0x66 f
"  @@ "_b,
" @   "_b,
"@@@@@"_b,
" @   "_b,
" @   "_b,
},
{ // 0x67 g
"  @@@"_b,
" @  @"_b,
"  @@@"_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x68 h
"@    "_b,
"@    "_b,
"@@@@ "_b,
"@   @"_b,
"@   @"_b,
},
{ // 0x69 i
"  @  "_b,
"     "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x6a j
"  @  "_b,
"     "_b,
"  @  "_b,
"  @  "_b,
"@@   "_b,
},
{ // 0x6b k
"@    "_b,
"@  @ "_b,
"@ @  "_b,
"@@@  "_b,
"@  @@"_b,
},
{ // 0x6c l
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @@ "_b,
},
{ // 0x6d m
"     "_b,
"     "_b,
"@@@@ "_b,
"@ @ @"_b,
"@ @ @"_b,
},
{ // 0x6e n
"     "_b,
"     "_b,
"@@@@ "_b,
"@   @"_b,
"@   @"_b,
},
{ // 0x6f o
"     "_b,
"     "_b,
" @@@ "_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x70 p
"     "_b,
" @@@ "_b,
"@   @"_b,
"@@@@ "_b,
"@    "_b,
},
{ // 0x71 q
"     "_b,
" @@@ "_b,
"@   @"_b,
" @@@@"_b,
"    @"_b,
},
{ // 0x72 r
"     "_b,
"     "_b,
"@ @@ "_b,
"@@   "_b,
"@    "_b,
},
{ // 0x73 s
"     "_b,
" @@@ "_b,
" @   "_b,
"  @  "_b,
"@@@  "_b,
},
{ // 0x74 t
"     "_b,
" @   "_b,
"@@@@@"_b,
" @   "_b,
" @@@ "_b,
},
{ // 0x75 u
"     "_b,
"     "_b,
"@   @"_b,
"@   @"_b,
" @@@ "_b,
},
{ // 0x76 v
"     "_b,
"     "_b,
"@   @"_b,
" @ @ "_b,
"  @  "_b,
},
{ // 0x77 w
"     "_b,
"     "_b,
"@ @ @"_b,
"@ @ @"_b,
" @ @@"_b,
},
{ // 0x78 x
"     "_b,
"     "_b,
"@  @ "_b,
" @@  "_b,
"@  @ "_b,
},
{ // 0x79 y
"     "_b,
"@   @"_b,
" @ @ "_b,
"  @  "_b,
"@@   "_b,
},
{ // 0x7a z
"     "_b,
"@@@@ "_b,
"  @  "_b,
" @   "_b,
"@@@@ "_b,
},
{ // 0x7b {
"  @@ "_b,
"  @  "_b,
"@@   "_b,
"  @  "_b,
"  @@ "_b,
},
{ // 0x7c |
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
"  @  "_b,
},
{ // 0x7d }
" @@  "_b,
"  @  "_b,
"   @@"_b,
"  @  "_b,
" @@  "_b,
},
{ // 0x7e ~
"@@@@@"_b,
"     "_b,
"     "_b,
"     "_b,
"     "_b,
},





}; 

