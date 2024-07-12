-- <pre> Radio-Frequency Detector Module (A3038DM) Firmware, Toplevel Unit

-- [18-JUN-24] Based on P3038DM V8.1, This standalone detector module carries its 
-- own 10-MHz oscillator and acts as a transmitter-on test circuit for surgeries 
-- and storage rooms. We switch DMCK to the on-board oscillator footprint that is
-- available on the A303801A printed circuit board. Remove HIDE and SHOW. Eliminate
-- indication of message buffer overflow and underflow.

-- [11-JUL-24] Send the indicator lamps 1..5 to DC1..DC5. Eliminate unused processes
-- and buffers.

-- Global Constants
library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		Q : in std_logic; -- Comparator Output from Detector
		SDO : in std_logic;  -- Serial Data Out from ADC
		SCK : inout std_logic; -- Serial Clock for ADC
		NCS : inout std_logic; -- Negated Chip Select for ADC
		RESET : in std_logic; -- Reset from master (DC0)
		DC : out std_logic_vector(5 downto 1); -- Indicators (DC1-DC5)
		DMCK : in std_logic; -- 10-MHz Clock (DC7)
		LED : inout std_logic_vector(5 downto 1); -- Indictors
		TP : out std_logic_vector(4 downto 1) -- Test Points
	);
	
	constant power_calibration : integer := 0;
	constant run_led_num : integer := 1;
	constant err_led_num : integer := 2;
	constant inc_led_num : integer := 3;
	constant rcv_led_num : integer := 4;
	constant pwr_led_num : integer := 5;
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Message Decoder Signals
	signal CONTINUE : std_logic; -- Tell message detector to continue detection.
	signal RSTDCR : std_logic; -- Reset the decoder.
	signal INCOMING : std_logic; -- The message detector is receiving a message.
	signal RECEIVED : std_logic; -- The message detector has received a complete message.
	signal message_id : integer range 0 to 255; -- id of received message.
	signal message_data : std_logic_vector (15 downto 0); -- contents of received message.

-- Clock and Timing Signals.
	signal CK : std_logic; -- Decoder Clock 40 MHz
	signal IOCK : std_logic; -- Input-Output Clock 20 MHz
	signal SLWCK : std_logic; -- Slow Clock 10 kHz
	signal LOCK : std_logic; -- PLL Lock
	signal clock_divA : std_logic_vector(7 downto 0);
	
-- ADC Signals
	signal pwr, pwr_rcv, pwr_intensity : std_logic_vector(7 downto 0);
		
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;
	
-- Message FIFO
	signal received_record, recalled_record : std_logic_vector(31 downto 0);
	signal FIFO_EMPTY, FIFO_FULL, WRMSG, RDMSG : std_logic;
	signal FIFO_EMPTY_ERR, FIFO_FULL_ERR : boolean;

begin
	-- The message decoder runs off 40 MHz. We receive 8 MHz on DMCK and use 
	-- a PLL to multiply to 40 MHz.
	Clock : entity P3038DM_PLL port map (
		CLKI => DMCK,
		CLKOP => CK,
		LOCK => LOCK
	);

	-- We provide IOCK (20 MHz), and SLWCK (10 kHz).
	Slow_Clocks : process (CK) is
	variable div1 : integer range 0 to 63;
	begin
		if rising_edge(CK) then
			clock_divA <= std_logic_vector(unsigned(clock_divA)+1);
			IOCK <= to_std_logic(IOCK = '0');
		end if;
		
		if rising_edge(clock_divA(7)) then
			if div1 = 15 then
				div1 := 0;
			else
				div1 := div1 + 1;
			end if;
			SLWCK <= to_std_logic(div1 < 9);
		end if;
	end process;
	
	-- The Message Decoder watches the incoming logic sequence and looks for
	-- messages. When it sees an incoming message, it asserts INCOMING. When
	-- it sees a complete message, it asserts RECEIVED. It provides the message
	-- id and data for storage, and waits until it receives RST before
	-- clearing its id and data values and looking for the next message. If
	-- it receives RST at any other time, it returns to its rest state, waiting
	-- for a new message to arrive.
	Decoder : entity message_decoder port map (
		CK => CK,-- Clock, 40.0 MHz
		Q => Q, -- Demodulator Output
		RST => RSTDCR, -- Reset the Decoder
		INCOMING => INCOMING, -- Message Incoming
		RECEIVED => RECEIVED, -- Message Received
		message_id => message_id, -- Eight-bit message ID
		message_data => message_data -- Message Data
	);
		
	-- We sample the power detector output and read out the ADC when we
	-- see INCOMING asserted. We acquire the power detector output voltage 
	-- as soon as we see INCOMING asserted, by driving the ADC chip select 
	-- input low (asserting !CS). We then wait until RECEIVED is asserted, 
	-- at which time it is safe to read out the ADC without disturbing
	-- message reception. We generate SCK at 20 MHz and read out three dummy
	-- bits, eight data bits, and three terminating bits. The ADC transitions
	-- to input tracking after the third terminating bit. Once the readout is
	-- complete, we wait for INCOMING to be unasserted. The PRDY flag 
	-- indicates that a new power measurement is ready, and will be asserted 
	-- until the power readout returns to rest.
	Power_Readout : process (IOCK) is
	constant pwr_end : integer := 31;
	constant wait_end : integer := 37;
	variable state, next_state : integer range 0 to 63;
	begin
		if RESET = '1' then
			state := 0;
			SCK <= '1';
			NCS <= '1';
			pwr <= "00000000";
		elsif rising_edge(IOCK) then
			next_state := state;
			if state = 0 then
				if (INCOMING = '1') then 
					next_state := 1;
				else
					next_state := 0;
				end if;
			elsif state = 1 then
				if (INCOMING = '0') then
					next_state := 0;
				elsif (RECEIVED = '1') then
					next_state := 2;
				else
					next_state := 1;
				end if;
			elsif state = pwr_end then
				if (INCOMING = '0') then
					next_state := state + 1;
				else
					next_state := pwr_end;
				end if;
			elsif state = wait_end then
				next_state := 0;
			else
				next_state := state + 1;
			end if;
			case state is
				when 0  => SCK <= '1'; NCS <= '1';  -- Idle
				when 1  => SCK <= '1'; NCS <= '0';  -- Acquire input voltage
				when 2  => SCK <= '0'; NCS <= '0';  -- Initiate conversion
				when 3  => SCK <= '1'; NCS <= '0';
				when 4  => SCK <= '0'; NCS <= '0';  -- Continue conversion
				when 5  => SCK <= '1'; NCS <= '0';
				when 6  => SCK <= '0'; NCS <= '0';  -- Complete conversion
				when 7  => SCK <= '1'; NCS <= '0';
				when 8  => SCK <= '0'; NCS <= '0';  -- Bit 7
				when 9  => SCK <= '1'; NCS <= '0';
				when 10 => SCK <= '0'; NCS <= '0';  -- Bit 6
				when 11 => SCK <= '1'; NCS <= '0';
				when 12 => SCK <= '0'; NCS <= '0';  -- Bit 5
				when 13 => SCK <= '1'; NCS <= '0';
				when 14 => SCK <= '0'; NCS <= '0';  -- Bit 4
				when 15 => SCK <= '1'; NCS <= '0';
				when 16 => SCK <= '0'; NCS <= '0';  -- Bit 3
				when 17 => SCK <= '1'; NCS <= '0';
				when 18 => SCK <= '0'; NCS <= '0';  -- Bit 2
				when 19 => SCK <= '1'; NCS <= '0';
				when 20 => SCK <= '0'; NCS <= '0';  -- Bit 1
				when 21 => SCK <= '1'; NCS <= '0';
				when 22 => SCK <= '0'; NCS <= '0';  -- Bit 0
				when 23 => SCK <= '1'; NCS <= '0';
				when 24 => SCK <= '0'; NCS <= '0';  -- Set up next cycle
				when 25 => SCK <= '1'; NCS <= '0';
				when 26 => SCK <= '0'; NCS <= '0';  -- Set up next cycle
				when 27 => SCK <= '1'; NCS <= '0';
				when 28 => SCK <= '0'; NCS <= '0';  -- Track input
				when 29 => SCK <= '1'; NCS <= '0';
				when 30 => SCK <= '1'; NCS <= '1';  -- Terminate cycle
				when others => SCK <= '1'; NCS <= '1'; -- Idle
			end case;
			case state is
				when 9 | 11 | 13 | 15 | 17 | 19 | 21 | 23 => 
					pwr(7 downto 1) <= pwr(6 downto 0);
					pwr(0) <= SDO;
				when others =>
					pwr <= pwr;
			end case;
			if state = pwr_end then
				pwr_rcv <= pwr;
				RSTDCR <= '1';
			else
				RSTDCR <= '0';
				pwr_rcv <= pwr_rcv;
			end if;
			state := next_state;
		end if;
	end process;
		
	-- The run indicator glows steady if power is on, logic is programmed,
	-- and the clocks are running.
	Run_Indicator : process (SLWCK) is
	variable counter : integer range 0 to 15;
	variable run_led_on : boolean;
	begin
		if RESET = '1' then
			counter := 0;
			run_led_on := false;
		elsif rising_edge(SLWCK) then
			counter := counter +1;
		end if;
		
		run_led_on := (counter rem 4) = 3;
			
		LED(run_led_num) <= to_std_logic(run_led_on);
	end process;
	
	-- The status indicator turns on when we have no PLL lock.
	Status_Indicator : process (RESET) is
	variable counter : integer range 0 to 15;	variable err_led_on : boolean;
	begin
		if RESET = '1' then
			LED(err_led_num) <= '0';
		else
			LED(err_led_num) <= not LOCK;
		end if;
	end process;
	
	-- Indoming LED indicates message is being decoded now.
	Incoming_Indicator : process (RESET,CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if RESET = '1' then
			LED(inc_led_num) <='0';	
		elsif rising_edge(CK) then
			if (counter = 0) then
				if (INCOMING = '1') then
					counter := 1;
				else
					counter := 0;
				end if;
				LED(inc_led_num) <='0';	
			else	
				if (counter = pulse_length) then
					counter := 0;
				else
					counter := counter + 1;
				end if;
				LED(inc_led_num) <= '1';				
			end if;
		end if;
	end process;
	
	-- Received LED indicates complete message received.
	Received_Indicator : process (RESET,CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if RESET = '1' then
			LED(rcv_led_num) <= '0';
		elsif rising_edge(CK) then
			if (counter = 0) then
				if (RECEIVED = '1') then
					counter := 1;
				else
					counter := 0;
				end if;
				LED(rcv_led_num) <= '0';
			else
				if (counter = pulse_length) then
					counter := 0;
				else
					counter := counter + 1;
				end if;
				LED(rcv_led_num) <= '1';				
			end if;
		end if;
	end process;

	-- Power Indicator Look-Up Table. Maps detector power values to display
	-- intensity values.
	Power_Map : entity P3038DM_PWMAP port map (
		Address => pwr_rcv,
		Q => pwr_intensity
	);

	-- Power LED brightness proportional to the log of most recently-received 
	-- message's power. We use the slow clock to generate a lamp signal with
	-- duty cycle proportional to log power.
	Power_Indicator : process (RESET,SLWCK) is 
	variable counter : integer range 0 to 255;
	begin
		if RESET = '1' then
			LED(pwr_led_num) <= '0';
		elsif rising_edge(SLWCK) then
			if counter < to_integer(unsigned(pwr_intensity)) then
				LED(pwr_led_num) <= '1';	
			else
				LED(pwr_led_num) <= '0';
			end if;
			counter := counter + 1;
		end if;
	end process;
	
	-- External indicators are equal to the on-board indicators.
	External_Indicators : process (RESET) is
	begin
		for i in 1 to 5 loop
			DC(i) <= LED(i);
		end loop;
	end process;
	
	-- Test Points, which are available externally.
	TP(1) <= CK;
	TP(2) <= RECEIVED;
	TP(3) <= SDO;
	TP(4) <= RSTDCR;
end behavior;