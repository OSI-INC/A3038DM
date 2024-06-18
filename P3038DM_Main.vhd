-- <pre> Radio-Frequency Detector Module (A3038DM) Firmware, Toplevel Unit

-- [18-JUN-24] Based on P3038DM V8.1, This standalone detector module carries its 
-- own 10-MHz oscillator and acts as a transmitter-on test circuit for surgeries 
-- and storage rooms. We switch DMCK to the on-board oscillator footprint that is
-- available on the A303801A printed circuit board. Remove HIDE and SHOW. Eliminate
-- indication of message buffer overflow and underflow.

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
		DSD : in std_logic; -- Data Strobe Downstream
		ddb : out std_logic_vector(7 downto 0); -- Data Downstream Bus
		DSU : out std_logic; -- Data Strobe Upstream
		dub : in std_logic_vector(7 downto 0); -- Data Upstream Bus
		RESET : in std_logic; -- Reset from master (DC0)
		DRC : in std_logic; -- Detector Readout Complete from master (DC1)
		GERR : inout std_logic; -- Global detector module error flag (DC2)
		GINC : inout std_logic; -- Global incoming message flag (DC3)
		GRCV : inout std_logic; -- Global message received flag (DC4)
		DMCK : in std_logic; -- Detector Module Clock (10 MHz Oscillator)
		LED : out std_logic_vector(5 downto 1); -- Indictors Lamps
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

-- Detector Control Bus Signals
	signal DMERR : std_logic; -- Detector module error, to master.
	signal DSD_sync, GINC_sync, GRCV_sync, DRC_sync : std_logic; -- Synchronized inputs.

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
	signal LCK : std_logic; -- Lamp Clock 10 Hz
	signal LOCK : std_logic; -- PLL Lock
	signal clock_divA, clock_divB : std_logic_vector(7 downto 0);
	
-- ADC Signals
	signal pwr, pwr_rcv, pwr_intensity : std_logic_vector(7 downto 0);
	signal PRDY : boolean;		
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

	-- Synchronize control inputs with respect to the rising edge of CK. Generate 
	-- IOCK at 20 MHz. Apply de-bounce to incoming signals that fall slowly so as
	-- to guarantee only one falling edge.
	Synchronizer : process (CK) is
	constant max_state : integer := 15;
	variable inc_state, next_inc_state : integer range 0 to max_state;
	variable rcv_state, next_rcv_state : integer range 0 to max_state;
	begin
		-- Generate 20-MHz Input-Output Clock.
		if rising_edge(CK) then
			IOCK <= to_std_logic(IOCK = '0');
		end if;
		
		-- Synchronize inputs asserted by controller.
		if rising_edge(CK) then
			DSD_sync <= DSD;
			DRC_sync <= DRC;
		end if;
		
		-- Synchronize GINC, debounce falling edge.
		if RESET = '1' then
			inc_state := 0;
			GINC_sync <= '0';
		elsif rising_edge(CK) then
			if inc_state = 0 then
				if GINC = '0' then
					GINC_sync <= '0';
					next_inc_state := 0;
				else
					GINC_sync <= '1';
					next_inc_state := 1;
				end if;
			elsif inc_state = 1 then
				GINC_sync <= '1';
				if GINC = '0' then
					next_inc_state := 2;
				else
					next_inc_state := 1;
				end if;
			elsif inc_state = max_state then
				GINC_sync <= '0';
				next_inc_state := 0;
			else
				GINC_sync <= '0';
				next_inc_state := inc_state + 1;
			end if;
			inc_state := next_inc_state;
		end if;
		
		-- Synchronize GRCV, debounce falling edge.
		if RESET = '1' then
			rcv_state := 0;
			GRCV_sync <= '0';
		elsif rising_edge(CK) then
			if rcv_state = 0 then
				if GRCV = '0' then
					GRCV_sync <= '0';
					next_rcv_state := 0;
				else
					GRCV_sync <= '1';
					next_rcv_state := 1;
				end if;
			elsif rcv_state = 1 then
				GRCV_sync <= '1';
				if GRCV = '0' then
					next_rcv_state := 2;
				else
					next_rcv_state := 1;
				end if;
			elsif rcv_state = max_state then
				GRCV_sync <= '0';
				next_rcv_state := 0;
			else
				GRCV_sync <= '0';
				next_rcv_state := rcv_state + 1;
			end if;
			rcv_state := next_rcv_state;
		end if;
	end process;
	
	-- We use the lamp clock to provide human-visible signalling with lamps.
	-- Our divider runs of CK, which is 40 MHz. We divide CK by 256 twice to 
	-- get 610 Hz. We divide by 61 to get 10 Hz to control lamp signals. We 
	-- divide CK by 256 once to get 156 kHz, then divide by 16 to get 10 kHz.
	Slow_Clocks : process (CK) is
	variable div1,div2 : integer range 0 to 63;
	begin
		if rising_edge(CK) then
			clock_divA <= std_logic_vector(unsigned(clock_divA)+1);
		end if;
		if rising_edge(clock_divA(7)) then
			clock_divB <= std_logic_vector(unsigned(clock_divB) + 1);
			if div1 = 15 then
				div1 := 0;
			else
				div1 := div1 + 1;
			end if;
			SLWCK <= to_std_logic(div1 < 9);
		end if;
		if rising_edge(clock_divB(7)) then
			if div2 = 60 then
				div2 := 0;
			else
				div2 := div2 + 1;
			end if;
			LCK <= to_std_logic(div2 < 30);
		end if;
	end process;
	
	-- The Message Decoder watches the incoming logic sequence and looks for
	-- messages. When it sees an incoming message, it asserts INCOMING. When
	-- it sees a complete message, it asserts RECEIVED. It provides the message
	-- id and data for storage, and waits until it receives RST before
	-- clearing its id and data values and looking for the next message. If
	-- it receives RST at any other time, it returns to its reast state, waiting
	-- for a new message to arrive.
	Decoder : entity message_decoder port map (
		CK => CK,-- Clock, 40.000 MHz
		Q => Q, -- Demodulator Output
		RST => RSTDCR, -- Reset the Decoder
		INCOMING => INCOMING, -- Message Incoming
		RECEIVED => RECEIVED, -- Message Received
		message_id => message_id, -- Eight-bit message ID
		message_data => message_data -- Message Data
	);
		
	-- The Detector Control process operates the various detector control
	-- lines that are shared by all detector modules and the master. In
	-- particular, there are three global shared lines that each detector
	-- can assert or leave tri-state. These are the global error, incoming,
	-- and received lines.
	Detector_Control : process (CK) is
	begin
		if rising_edge(CK) then
			if DMERR = '1' then
				GERR <= '1';
			else
				GERR <= 'Z';
			end if;
			
			if INCOMING = '1' then
				GINC <= '1';
			else
				GINC <= 'Z';
			end if;
			
			if RECEIVED = '1' then
				GRCV <= '1';
			else
				GRCV <= 'Z';
			end if;
		end if;
	end process;
	
	-- The Message FIFO receives new 32-bit message records and allows the
	-- them to be read out later by the detector module daisy chain. 
	Message_FIFO : entity P3038DM_FIFO port map (
		Data => received_record,
        WrClock => CK,
        RdClock => CK,
        WrEn => WRMSG,
        RdEn => RDMSG,
        Reset => RESET,
        RPReset => RESET,
        Q => recalled_record,
        Empty => FIFO_EMPTY,
        Full => FIFO_FULL
	);	
	
	-- We sample the power detector output and read out the ADC when we
	-- see Global Incoming (GINC) asserted. We acquire the power detector
	-- output voltage as soon as we see GINC asserted, by driving the ADC
	-- chip select input low (asserting !CS). We then wait until GRCV is
	-- asserted, at which time it is safe to read out the ADC without disturbing
	-- message reception. We generate SCK at 20 MHz and read out three dummy
	-- bits, eight data bits, and three terminating bits. The ADC transitions
	-- to input tracking after the third terminating bit. Once the readout is
	-- complete, we wait for GINC to be unasserted, and then wait a little 
	-- longer to make sure that GINC has settled. The GINC line is a global
	-- signal with pull-down. It takes some time to settle back to zero, and 
	-- transitions in nearby logic signals can cause it to glitch back up.
	-- After the wait, we return to the rest state. The PRDY flag indicates that
	-- a new power measurement is ready, and will be asserted until the power
	-- readout returns to rest.
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
			PRDY <= false;
		elsif rising_edge(IOCK) then
			next_state := state;
			if state = 0 then
				if (GINC_sync = '1') then 
					next_state := 1;
				else
					next_state := 0;
				end if;
			elsif state = 1 then
				if (GINC_sync = '0') then
					next_state := 0;
				elsif (GRCV_sync = '1') then
					next_state := 2;
				else
					next_state := 1;
				end if;
			elsif state = pwr_end then
				if (GINC_sync = '0') then
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
			PRDY <= (state >= pwr_end);
			state := next_state;
		end if;
	end process;
	
	-- The Message Recorder saves 32-bit message records in the FIFO. The
	-- top eight bits are the message id, the next sixteen are the message
	-- data, and the final eight are the power. If this detector did not
	-- receive the message, the id and data are all zero.
	Message_Recorder : process (CK,RESET) is
	variable state, next_state : integer range 0 to 3;
	begin
		if RESET = '1' then
			state := 0;
			RSTDCR <= '1';
			pwr_rcv <= "00000000";
		elsif rising_edge(CK) then
			next_state := state;
			RSTDCR <= '0';
			WRMSG <= '0';
			case state is
				when 0 =>
					if (GRCV_sync = '1') and PRDY then 
						next_state := 1;
					end if;
				when 1 =>
					next_state := 2;
					WRMSG <= '1';
					pwr_rcv <= pwr;
				when 2 =>
					RSTDCR <= '1';
					if GRCV_sync = '0' then
						next_state := 3;
					else
						next_state := 2;
					end if;
				when 3 =>
					RSTDCR <= '1';
					next_state := 0;
			end case;
			state := next_state;
		end if;
		
		if RECEIVED = '1' then
			received_record(31 downto 24) <= std_logic_vector(to_unsigned(message_id,8));
			received_record(23 downto 8) <= message_data;
		else
			received_record(31 downto 24) <= "00000000";
			received_record(23 downto 8) <= "0000000000000000";
		end if;
		received_record(7 downto 0) <= pwr;
	end process;
	
	-- The Message Reader responds to the readout process on the detector module
	-- daisy chain. From its rest state, it reads a message from the FIFO after
	-- seeing Data Strobe Downstream (DSD) from the module closer to the master.
	-- All modules are receiving the same DSD and reading their records of the 
	-- same message in resonse to the same DSD pulse. The first DSD cycle gives
	-- the detectors a chance to establish if they are the highest-power receiver
	-- in comparison to all upstream modules, and we set saved_local_best to mark
	-- this comparison. The next DSD cycle invites the highest-power receivers to
	-- send their message_id downstream. Other detectors relay the upstream data
	-- to their downstream. The next DSD cycle invites the highest-power receivers
	-- to do the same for the top eight data bits. The next cycle is for the bottom
	-- eight data bits. The cycle after that is blocked from passing upstream: each
	-- detector prevents DSD from being sent to DSU, and responds to the cycle 
	-- itself, with its own power measurement. Thereafter, it permits all subsequent
	-- DSD cycles to propagate upstream, and forwards the upstream data to the
	-- downstream data. In this way the master receives the id, high data, low data,
	-- and then power from the first to the last module in the daisy chain. The master
	-- asserte Detector Read Complete (DRC) to send all Message Readers back to their
	-- rest state, ready to read out the next message from the FIFO.
	Message_Reader : process (CK,RESET) is
	variable state, next_state : integer range 0 to 31;
	variable recalled_id,recalled_pwr,upstream_value : integer range 0 to 255;
	variable recalled_hi,recalled_lo : std_logic_vector(7 downto 0);
	variable ddb_select : integer range 0 to 7;
	constant select_zero : integer := 0;
	constant select_dub : integer := 1;
	constant select_pwr : integer := 2;
	constant select_hi : integer := 3;
	constant select_lo : integer := 4;
	constant select_id : integer := 5;
	constant max_pwr : integer := 6;
	variable local_best, local_best_saved : boolean;
	variable ds_forward : boolean;
	begin
		recalled_id := to_integer(unsigned(recalled_record(31 downto 24)));
		recalled_hi := recalled_record(23 downto 16);
		recalled_lo := recalled_record(15 downto 8);
		recalled_pwr := to_integer(unsigned(recalled_record(7 downto 0)));
		upstream_value := to_integer(unsigned(dub));
		local_best := (upstream_value <= recalled_pwr) and (recalled_id /= 0);
	
		if (RESET = '1') or (DRC_sync = '1') then
			state := 0;
			RDMSG <= '0';
		elsif rising_edge(CK) then
			next_state := state;
			RDMSG <= '0';
			case state is 
				when 0 =>
					if DSD_sync = '1' then next_state := 1; end if;
					local_best_saved := false;
					ddb_select := select_zero;
					ds_forward := true;
				when 1 =>
					next_state := 2;
					ddb_select := select_zero;
					ds_forward := true;
					RDMSG <= '1';
				when 2 => 
					if DSD_sync = '0' then next_state := 3; end if;
					local_best_saved := local_best;
					ddb_select := max_pwr;
					ds_forward := true;
				when 3 =>
					if DSD_sync = '1' then next_state := 4; end if;
					ddb_select := select_zero;
					ds_forward := true;
				when 4 => 
					if DSD_sync = '0' then next_state := 5; end if;
					if local_best_saved then
						ddb_select := select_id;
					else
						ddb_select := select_dub;
					end if;
					ds_forward := true;
				when 5 =>
					if DSD_sync = '1' then next_state := 6; end if;
					ddb_select := select_zero;
					ds_forward := true;
				when 6 =>
					if DSD_sync = '0' then next_state := 7; end if;
					if local_best_saved then
						ddb_select := select_hi;
					else
						ddb_select := select_dub;
					end if;
					ds_forward := true;
				when 7 => 
					if DSD_sync = '1' then next_state := 8; end if;
					ddb_select := select_zero;
					ds_forward := true;
				when 8 =>
					if DSD_sync = '0' then next_state := 9; end if;
					if local_best_saved then
						ddb_select := select_lo;
					else
						ddb_select := select_dub;
					end if;
					ds_forward := true;
				when 9 => 
					if DSD_sync = '1' then next_state := 10; end if;
					ddb_select := select_zero;
					ds_forward := false;
				when 10 =>
					if DSD_sync = '0' then next_state := 11; end if;
					ddb_select := select_pwr;
					ds_forward := false;
				when 11 => 
					if DSD_sync = '1' then next_state := 12; end if;
					ddb_select := select_zero;
					ds_forward := true;
				when 12 =>
					ddb_select := select_dub;
					ds_forward := true;
				when others =>
					next_state := 0;
					ds_forward := true;
			end case;
			state := next_state;
		end if;
		
		case ddb_select is
			when select_zero => ddb <= "00000000";
			when select_dub => ddb <= dub;
			when select_pwr => ddb <= recalled_record(7 downto 0);
			when select_hi => ddb <= recalled_hi;
			when select_lo => ddb <= recalled_lo;
			when select_id => ddb <= recalled_record(31 downto 24);
			when max_pwr => 	
				if local_best then
					ddb <= recalled_record(7 downto 0);
				else
					ddb <= dub;
				end if;
			when others => ddb <= "00000000";
		end case;
		
		if ds_forward then
			DSU <= DSD_sync;
		else
			DSU <= '0';
		end if;
	end process;
	
	-- The error detector looks for failure to lock.
	Error_Detection : process (CK,RESET) is
	begin
		DMERR <= to_std_logic(LOCK = '0');
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
	
	-- The status indicator is off when all is well, and flashes 
	-- to indicate error conditions. In this standalone code, the
	-- only error is failure to get 40 MHz from 10 MHz with the PLL.
	Status_Indicator : process (LCK) is
	variable counter : integer range 0 to 15;	variable err_led_on : boolean;
	begin
		if rising_edge(LCK) then
			counter := counter + 1;
			err_led_on := false;
		end if;
		
		err_led_on := (LOCK = '0');
		
		LED(err_led_num) <= to_std_logic(err_led_on);
	end process;
	
	-- Indoming LED indicates message is being decoded now.
	Incoming_Indicator : process (CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if rising_edge(CK) then
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
	Received_Indicator : process (CK) is 
	variable counter : integer range 0 to 65535;
	constant pulse_length : integer := 32768;
	begin
		if rising_edge(CK) then
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
	Power_Indicator : process (SLWCK) is 
	variable counter : integer range 0 to 255;
	begin
		if rising_edge(SLWCK) then
			if counter < to_integer(unsigned(pwr_intensity)) then
				LED(pwr_led_num) <= '1';	
			else
				LED(pwr_led_num) <= '0';
			end if;
			counter := counter + 1;
		end if;
	end process;
	
	TP(1) <= SDO;
	TP(2) <= GINC_sync;
	TP(3) <= GRCV_sync;
	TP(4) <= DSD;
end behavior;