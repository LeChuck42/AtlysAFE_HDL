library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is
	port (
		CLK_FPGA:       in  std_logic;
		
		-- CLOCK SYNTH
		CLK_REFSEL:     out std_logic;
		CLK_PDN:        out std_logic;
		CLK_SYNC:       out std_logic;
		
		CLK_SPI_LE:     out std_logic;
		CLK_SPI_CLK:    out std_logic;
		CLK_SPI_MOSI:   out std_logic;
		CLK_SPI_MISO:   in  std_logic;
		
		-- CLOCK MUX
		CLKIN_SEL0:     out std_logic;
		CLKIN_SEL1:     out std_logic;
		CLKOUT_TYPE0:   out std_logic;
		CLKOUT_TYPE1:   out std_logic;
		
		-- MUX INTERFACE
		MUX_CLK:        in  std_logic;
		MUX_IN:         in  std_logic;
		MUX_OUT:        out std_logic;
		
		-- LDO
		ENAB_3V3_LDO:   out std_logic;
		ENAB_DAC_LDO:   out std_logic;
		FPGA_PGOOD:     in  std_logic;
		
		-- ADC
		PDN_IN1:        out std_logic;
		PDN_IN2:        out std_logic;
		
		ADC_PDNA:       out std_logic;
		ADC_PDNB:       out std_logic;
		ADC_RESET:      out std_logic;
		
		ADC_SPI_CS:     out std_logic;
		ADC_SPI_SDATA:  inout std_logic;
		ADC_SPI_CLK:    out std_logic;
		
		-- DAC
		DAC_TXEN:       out std_logic;
		DAC_ALARM:      in  std_logic;
		DAC_REFCLK:     out std_logic;
		
		DAC_SPI_CS:     out std_logic;
		DAC_SPI_SDIO:   inout std_logic;
		DAC_SPI_CLK:    out std_logic;
		
		-- USB SERIAL I/O
		SIO_RX:         in  std_logic;
		SIO_TX:         out std_logic;
		SIO_RTSn:       in  std_logic;
		SIO_CTSn:       out std_logic;
		SIO_DTRn:       in  std_logic;
		SIO_DSRn:       out std_logic;
		SIO_DCDn:       out std_logic;
		
		-- OTHER
		LED:            out std_logic_vector(3 downto 0);
		GPIO:           inout std_logic_vector(13 downto 0));
end entity main;

architecture rtl of main is

component pll_mux
	port (CLKI: in  std_logic; CLKOP: out  std_logic; 
	CLKOS: out  std_logic; CLKOS2: out std_logic;
	LOCK: out  std_logic);
end component;

component rx_mux
	port (alignwd: in  std_logic; clk: in  std_logic; 
		clk_s: in  std_logic; del: in  std_logic_vector(4 downto 0); 
		init: in  std_logic; reset: in  std_logic; 
		rx_ready: out  std_logic; sclk: out  std_logic; 
		datain: in  std_logic_vector(0 downto 0); 
		q: out  std_logic_vector(7 downto 0));
end component;

component tx_mux
	port (clk_s: in  std_logic; clkop: in  std_logic; 
		clkos: in  std_logic; clkout: out  std_logic; 
		lock_chk: in  std_logic; reset: in  std_logic; 
		sclk: out  std_logic; tx_ready: out  std_logic; 
		dataout: in  std_logic_vector(7 downto 0); 
		dout: out  std_logic_vector(0 downto 0));
end component;

component spi_register
	generic (
		BITS:   natural := 16);
	
	port (
		reset:      in std_logic;
		spi_cs:     in std_logic;
		spi_clk:    in std_logic;
		spi_mosi:   in std_logic;
		spi_miso:   out std_logic;
		
		reg_out:    out std_logic_vector(BITS-1 downto 0));
end component;

signal spi_cs_fpga: std_logic;
signal spi_miso_fpga: std_logic;
signal spi_reg: std_logic_vector(15 downto 0);
signal spi_cs_adc: std_logic;
signal spi_cs_dac: std_logic;
signal spi_cs_clk: std_logic;
signal spi_clk: std_logic;
signal spi_mosi: std_logic;
signal spi_miso: std_logic;
signal spi_oe: std_logic;

signal mux_synced: std_logic;
signal sync_pattern: std_logic_vector(7 downto 0);

signal rx_mux_out: std_logic_vector(7 downto 0);
signal rx_mux_q: std_logic_vector(7 downto 0);
signal rx_mux_alignwd: std_logic;
signal rx_mux_clk_s: std_logic;
signal rx_mux_init: std_logic;
signal rx_mux_rx_ready: std_logic;
signal rx_mux_sclk: std_logic;
signal rx_mux_datain: std_logic_vector(0 downto 0);
signal rx_mux_delay: std_logic_vector(4 downto 0);


signal tx_mux_d: std_logic_vector(7 downto 0);
signal tx_mux_reg: std_logic_vector(7 downto 0);
signal tx_mux_clk_s: std_logic;
signal tx_mux_clkop: std_logic;
signal tx_mux_clkos: std_logic;
signal tx_mux_clkout: std_logic;
signal tx_mux_lock_chk: std_logic;
signal tx_mux_sclk: std_logic;
signal tx_mux_tx_ready: std_logic;
signal tx_mux_dout: std_logic_vector(0 downto 0);

signal mux_reset: std_logic;
signal reset, reset_buf, reset_async: std_logic;
signal sync_delay: std_logic;
signal clk_fpga_int: std_logic;
signal sync_mon_valid, sync_mon_expect, sync_mon_out: std_logic;
begin
	--pll_main_inst : pll_main
	--  port map (CLKI=>CLK_FPGA, CLKOP=>clk, LOCK=>pll_lock);
	
	DAC_REFCLK <= CLK_FPGA;
	
	rx_mux_datain(0) <= MUX_IN;
	MUX_OUT <= tx_mux_dout(0);
	
	rx_mux_delay <= (others => '0');
	
	pll_mux_inst : pll_mux
		port map (CLKI=>MUX_CLK, CLKOP=>tx_mux_clkop, CLKOS=>tx_mux_clkos, CLKOS2=>clk_fpga_int, LOCK=>tx_mux_lock_chk);

	tx_mux_clk_s <= clk_fpga_int;
	rx_mux_clk_s <= clk_fpga_int;
	mux_reset <= not tx_mux_lock_chk;
	rx_mux_init <= not mux_reset;
	
	
	rx_mux_inst: rx_mux
	port map (alignwd=>rx_mux_alignwd, clk=>MUX_CLK, clk_s=>rx_mux_clk_s, del(4 downto 0)=>rx_mux_delay, init=>rx_mux_init, reset=>mux_reset,
		rx_ready=>rx_mux_rx_ready, sclk=>rx_mux_sclk, datain(0 downto 0)=>rx_mux_datain, q(7 downto 0)=>rx_mux_q);

	tx_mux_inst: tx_mux
	port map (clk_s=>tx_mux_clk_s, clkop=>tx_mux_clkop, clkos=>tx_mux_clkos, clkout=>tx_mux_clkout, lock_chk=>tx_mux_lock_chk, 
		reset=>mux_reset, sclk=>tx_mux_sclk, tx_ready=>tx_mux_tx_ready, dataout(7 downto 0)=>tx_mux_d, dout(0 downto 0)=>tx_mux_dout);
	
	spi_reg_inst: spi_register
		generic map ( BITS => 16)
		port map(
			reset => mux_reset,
			spi_cs => spi_cs_fpga,
			spi_clk => spi_clk,
			spi_mosi => spi_mosi,
			spi_miso => spi_miso_fpga,
			reg_out => spi_reg);
	
	spi_miso <= spi_miso_fpga when spi_cs_fpga = '0' else 'Z';
	
	reset_async <= not FPGA_PGOOD;
	
	reset_sync_proc: process(reset_async, rx_mux_sclk)
	begin
		if reset_async = '1' then
			reset <= '1';
			reset_buf <= '1';
		elsif rising_edge(rx_mux_sclk) then
			reset <= reset_buf;
			reset_buf <= '0';
		end if;
	end process;
	
	mux_sync_func: process(mux_reset, rx_mux_sclk)
	begin
		if mux_reset = '1' then
			rx_mux_alignwd <= '0';
			mux_synced <= '0';
			sync_pattern <= x"01";
			sync_delay <= '1';
			sync_mon_out <= '0';
			sync_mon_valid <= '0';
			sync_mon_expect <= '0';
		elsif rising_edge(rx_mux_sclk) then
			sync_delay <= rx_mux_alignwd;
			rx_mux_alignwd <= '0';
			sync_mon_out <= not sync_mon_out;
			if mux_synced = '1' then
				sync_pattern <= x"01";
				if sync_mon_valid = '1' then
					if sync_mon_expect = rx_mux_q(7) then
						sync_mon_expect <= not sync_mon_expect;
					else
						mux_synced <= '0';
						sync_delay <= '1';
						sync_mon_valid <= '0';
					end if;
				elsif rx_mux_q(7) = '0' then
					sync_mon_valid <= '1';
					sync_mon_expect <= '1';
				end if;
			else
				-- not synced
				sync_mon_valid <= '0';
				if rx_mux_alignwd = '0' and sync_delay = '0'then
					if rx_mux_q /= x"81" and rx_mux_q /= x"01" then
						-- no sync pattern -> shift inputs
						sync_pattern <= x"01";
						rx_mux_alignwd <= '1';
					else
						-- sync pattern recognized, wait for master
						sync_pattern <= x"81";
						if rx_mux_q = x"81" and sync_pattern = x"81" then
							mux_synced <= '1';
						end if;
					end if;
				end if;
			end if;
		end if;
	end process;
	
	tx_mux_d <= tx_mux_reg when mux_synced = '1' else sync_pattern;
	rx_mux_out <= rx_mux_q when mux_synced = '1' and sync_mon_valid = '1' else "00011111";
	
	spi_cs_fpga <= rx_mux_out(4);
	spi_cs_adc <= rx_mux_out(3);
	spi_cs_dac <= rx_mux_out(2);
	spi_cs_clk <= rx_mux_out(1);
	spi_clk <= rx_mux_out(6);
	spi_mosi <= rx_mux_out(5);
	spi_oe <= rx_mux_out(0);
	
	tx_mux_reg(0) <= spi_miso;
	tx_mux_reg(1) <= DAC_ALARM;
	tx_mux_reg(6 downto 2) <= (others => '0');
	tx_mux_reg(7) <= sync_mon_out;
			
	CLK_REFSEL <= 'Z';
	CLK_PDN <= mux_synced;
	CLK_SYNC <= 'Z';
	
	CLK_SPI_LE <= spi_cs_clk;
	CLK_SPI_CLK <= spi_clk when spi_cs_clk = '0' else '0';
	CLK_SPI_MOSI <= spi_mosi when spi_cs_clk = '0' else '0';
	spi_miso <= CLK_SPI_MISO when spi_cs_clk = '0' else 'Z';
	
	CLKIN_SEL0 <= spi_reg(4);
	CLKIN_SEL1 <= spi_reg(5);
	CLKOUT_TYPE0 <= spi_reg(6);
	CLKOUT_TYPE1 <= spi_reg(7);
	
	ENAB_3V3_LDO <= '1';
	ENAB_DAC_LDO <= '1';
	
	PDN_IN1 <= spi_reg(0);
	PDN_IN2 <= spi_reg(1);
	
	ADC_PDNA <= spi_reg(2);
	ADC_PDNB <= spi_reg(3);
	ADC_RESET <= '1' when mux_synced = '0' else '0';
	
	ADC_SPI_CS <= spi_cs_adc;
	ADC_SPI_SDATA <= spi_mosi when spi_cs_adc = '0' and spi_oe = '0' else 'Z';
	ADC_SPI_CLK <= spi_clk when spi_cs_adc = '0' else '0';
	spi_miso <= ADC_SPI_SDATA when spi_cs_adc = '0' else 'Z';
	
	DAC_TXEN <= '0';
	
	DAC_SPI_CS <= spi_cs_dac;
	DAC_SPI_SDIO <= spi_mosi when spi_cs_dac = '0' and spi_oe = '0' else 'Z';
	DAC_SPI_CLK <= spi_clk when spi_cs_dac = '0';
	spi_miso <= DAC_SPI_SDIO when spi_cs_dac = '0' else 'Z';
	
	SIO_TX <= 'Z';
	SIO_CTSn <= 'Z';
	SIO_DSRn <= 'Z';
	SIO_DCDn <= 'Z';
	GPIO <= rx_mux_out(6 downto 0) & rx_mux_alignwd & mux_synced & sync_delay & sync_mon_out & sync_mon_valid & spi_miso & tx_mux_lock_chk;
	LED(0) <= mux_synced;
	LED(1) <= spi_reg(13);
	LED(2) <= spi_reg(14);
	LED(3) <= spi_reg(15);
end architecture rtl;
		
		
		