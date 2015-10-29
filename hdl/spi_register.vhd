library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_register is
	generic (
		BITS:	natural := 16);
	
	port (
		-- slave port
		reset:		in std_logic;
		spi_cs:		in std_logic;
		spi_clk:	in std_logic;
		spi_mosi:	in std_logic;
		spi_miso:	out std_logic;
		
		reg_out:	out std_logic_vector(BITS-1 downto 0));
		
end entity spi_register;

architecture rtl of spi_register is
	signal shift_reg: std_logic_vector(BITS-1 downto 0);
	signal latch: std_logic_vector(BITS-1 downto 0);
begin
	
	latch <= shift_reg when spi_cs = '1' else latch;
	reg_out <= latch;
	
	reg_proc: process(spi_clk, reset)
	begin
		if reset = '1' then
			shift_reg <= (others => '0');
			spi_miso <= '0';
		elsif rising_edge(spi_clk) then
			if spi_cs = '0' then
				spi_miso <= shift_reg(BITS-1);
				shift_reg <= shift_reg(BITS-2 downto 0) & spi_mosi;
			end if;
		end if;
	end process;
end architecture rtl;