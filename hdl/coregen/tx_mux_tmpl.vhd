-- VHDL module instantiation generated by SCUBA Diamond (64-bit) 3.6.0.83.4
-- Module  Version: 5.8
-- Thu Oct 29 19:51:48 2015

-- parameterized module component declaration
component tx_mux
    port (clk_s: in  std_logic; clkop: in  std_logic; 
        clkos: in  std_logic; clkout: out  std_logic; 
        lock_chk: in  std_logic; reset: in  std_logic; 
        sclk: out  std_logic; tx_ready: out  std_logic; 
        dataout: in  std_logic_vector(7 downto 0); 
        dout: out  std_logic_vector(0 downto 0));
end component;

-- parameterized module component instance
__ : tx_mux
    port map (clk_s=>__, clkop=>__, clkos=>__, clkout=>__, lock_chk=>__, 
        reset=>__, sclk=>__, tx_ready=>__, dataout(7 downto 0)=>__, dout(0 downto 0)=>__);