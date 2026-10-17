# Verilog-ELEKKI-Platform
Implement a Smart Traffic Light Priority Controller for a side road. The output signal side_green should be driven high (1) to grant a green signal to the side-road traffic under specific priority rules.

The side road gets a green light if:

1. An emergency vehicle (ambulance/fire engine) is detected on the side road (emergency = 1), OR

2. There is no vehicle traffic detected on the main road (car_main = 0) AND (a vehicle is waiting on the side road (car_side = 1) OR a pedestrian button was pressed (pedestrian = 1)).
```
module traffic_priority_controller (
    input  logic car_main,
    input  logic car_side,
    input  logic pedestrian,
    input  logic emergency,
    output logic side_green
);

    assign side_green = emergency | (~car_main & (car_side | pedestrian));

endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Implement a Smart Traffic Light Priority Controller for a side road. The output signal side_green should be driven high (1) to grant a green signal to the side-road traffic under specific priority rules.
The side road gets a green light if:
1. An emergency vehicle (ambulance/fire engine) is detected on the side road (emergency = 1), OR
2. There is no vehicle traffic detected on the main road (car_main = 0) AND (a vehicle is waiting on the side road (car_side = 1) OR a pedestrian button was pressed (pedestrian = 1)).
Use Boolean simplification to reduce the control logic into a minimal logic network.
Port Specifications
```
module traffic_priority_controller (
    input  logic car_main,
    input  logic car_side,
    input  logic pedestrian,
    input  logic emergency,
    output logic side_green
);

    // side_green = E + M'(S + P)
    assign side_green = emergency | (~car_main & (car_side | pedestrian));

endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Implement a BCD to 7-Segment LED Display Decoder supporting common-anode or common-cathode mapping with active-high outputs, blanking control, and invalid BCD detection.
A BCD to 7-Segment decoder maps a 4-bit Binary Coded Decimal input ($0$ to $9$) to a 7-bit output bus (seg[6:0]) representing segments a through g to display decimal digits on a standard display element.
Segment mapping bit positions: seg[6:0] = {g, f, e, d, c, b, a}
Operational Logic:

Blanking Control (blank == 1): Overrides all decode logic and turns off all segments (seg = 7'b000_0000), forcing invalid = 1'b0.
Valid BCD Inputs (bcd from 4'h0 to 4'h9):
- Decodes input value to light corresponding segment pattern (Active-HIGH, 1 = Segment ON, 0 = Segment OFF):
- 0 (4'h0): 7'b011_1111 (Segments a, b, c, d, e, f active)
- 1 (4'h1): 7'b000_0110 (Segments b, c active)
- 2 (4'h2): 7'b101_1011 (Segments a, b, d, e, g active)
- 3 (4'h3): 7 me_100_1111 / 7'b100_1111 (Segments a, b, c, d, g active)
- 4 (4'h4): 7'b110_0110 (Segments b, c, f, g active)
- 5 (4'h5): 7'b110_1101 (Segments a, c, d, f, g active)
- 6 (4'h6): 7'b111_1101 (Segments a, c, d, e, f, g active)
- 7 (4'h7): 7'b000_0111 (Segments a, b, c active)
- 8 (4'h8): 7'b111_1111 (All segments active)
- 9 (4'h9): 7'b110_1111 (Segments a, b, c, d, f, g active)
- Output flag invalid = 1'b0.
Invalid BCD Range (bcd from 4'hA to 4'hF):
- Forces all segments OFF (seg = 7'b000_0000).
- Asserts invalid detection flag invalid = 1'b1.
Port Specifications

Inputs:
- bcd (4-bit): Binary Coded Decimal input vector [3:0].
- blank (1-bit): Active-high display blanking input line.
Outputs:
- seg (7-bit): Active-high 7-segment display control bus {g, f, e, d, c, b, a}.
- invalid (1-bit): Flag asserted HIGH when input is outside valid BCD range ($10\text{--}15$).

```  
// BCD to 7-segment decoder, gate-level (from K-map simplification)
// seg[6:0] = {g, f, e, d, c, b, a}, active-HIGH
// A = bcd[3] (MSB), B = bcd[2], C = bcd[1], D = bcd[0] (LSB)
module bcd_to_7seg_gates (
    input  wire [3:0] bcd,
    input  wire       blank,
    output wire [6:0] seg,
    output wire       invalid
);
    wire A = bcd[3], B = bcd[2], C = bcd[1], D = bcd[0];
    wire en = ~blank;

    // a = A'C + A'BD + AB'C' + B'C'D'
    assign seg[0] = en & ((~A & C) | (~A & B & D) | (A & ~B & ~C) | (~B & ~C & ~D));

    // b = A'B' + B'C' + A'CD + A'C'D'
    assign seg[1] = en & ((~A & ~B) | (~B & ~C) | (~A & C & D) | (~A & ~C & ~D));

    // c = A'B + B'C' + A'D
    assign seg[2] = en & ((~A & B) | (~B & ~C) | (~A & D));

    // d = A'BC'D + A'B'C + A'CD' + AB'C' + B'C'D'
    assign seg[3] = en & ((~A & B & ~C & D) | (~A & ~B & C) | (~A & C & ~D) |
                          (A & ~B & ~C) | (~B & ~C & ~D));

    // e = A'CD' + B'C'D'
    assign seg[4] = en & ((~A & C & ~D) | (~B & ~C & ~D));

    // f = A'BC' + A'BD' + AB'C' + B'C'D'
    assign seg[5] = en & ((~A & B & ~C) | (~A & B & ~D) | (A & ~B & ~C) | (~B & ~C & ~D));

    // g = A'B'C + A'BC' + A'BD' + AB'C'
    assign seg[6] = en & ((~A & ~B & C) | (~A & B & ~C) | (~A & B & ~D) | (A & ~B & ~C));

    // invalid = AB + AC = A(B + C)
    assign invalid = en & A & (B | C);

endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Implement a Solar-Grid Hybrid Battery Charger Switch. The output signal charge_on should drive high (1) to engage the power stage charging relay for an energy storage bank.
The charger activates if the battery is NOT fully charged (batt_full = 0) AND either of the following conditions is satisfied:
1. Solar panel power is available (solar = 1).
2. Solar power is unavailable (solar = 0) BUT AC grid power is available (grid = 1) AND the battery has dropped to a low voltage state (batt_low = 1).
Perform Boolean simplification on this 4-variable state table to minimize logic gate count.
Port Specifications

Inputs:
- solar (1-bit): Solar panel power presence (1 = Solar power active).
- grid (1-bit): AC grid power presence (1 = Grid connected).
- batt_full (1-bit): Battery upper charge cutoff sensor (1 = Battery 100% full).
- batt_low (1-bit): Battery low-voltage warning threshold (1 = Critical discharge).
Outputs:
- charge_on (1-bit): Signal enabling the power electronic battery charger.
```
  module solar_charger_controller (
    input  logic solar,
    input  logic grid,
    input  logic batt_full,
    input  logic batt_low,
    output logic charge_on
);

    // charge_on = ~batt_full & (solar | (grid & batt_low))
    assign charge_on = ~batt_full & (solar | (grid & batt_low));

endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Implement an HVAC Energy-Saving Air Conditioner Controller. The output signal ac_on should go high (1) to activate the compressor unit based on room temperature and occupancy parameters.

The compressor turns ON if:

1. The ambient room temperature is higher than the setpoint (temp_high = 1).

2. The room is occupied (occupancy = 1).

3. No perimeter windows are open (window_open = 0).

4. The system is NOT operating in eco-lockout mode (eco_mode = 0), OR if eco_mode = 1, the temperature is critically high (crit_high = 1).

Simplify this multi-variable boolean rule using K-map or Boolean reduction techniques.
```
module hvac_ac_controller (
    input  logic temp_high,
    input  logic occupancy,
    input  logic window_open,
    input  logic eco_mode,
    input  logic crit_high,
    output logic ac_on
);

    // Write your K-map minimized Boolean logic here
    assign ac_on=temp_high&occupancy&(~window_open)&(~(eco_mode&~crit_high));

endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Implement a 4-to-1 Multiplexer. The circuit selects one of four 1-bit data inputs (d0, d1, d2, d3) based on a 2-bit selection input sel[1:0] and routes it to the single output out.

The multiplexing logic operates as follows:

sel = 2'b00 -> out = d0
sel = 2'b01 -> out = d1
sel = 2'b10 -> out = d2
sel = 2'b11 -> out = d3
Port Specifications
Inputs:
- d0, d1, d2, d3 (1-bit each): Data input lines.
- sel (2-bit): Selection line vector (sel[1:0]).
Outputs:
- out (1-bit): Multiplexed output signal
```
  module mux_4to1 (
    input  wire       d0,
    input  wire       d1,
    input  wire       d2,
    input  wire       d3,
    input  wire [1:0] sel,
    output reg        out
);
    // Write your Verilog implementation here
    wire A=sel[1],B=sel[0];
    assign out=(~A&~B&d0)|(~A&B&d1)|(A&~B&d2)|(A&B&d3);


endmodule
```
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
