-- VHDL module instantiation generated by SCUBA Diamond (64-bit) 3.6.0.83.4
-- Module  Version: 5.7
-- Thu Oct 29 19:34:09 2015

-- parameterized module component declaration
component pll_mux
    port (CLKI: in  std_logic; CLKOP: out  std_logic; 
        CLKOS: out  std_logic; CLKOS2: out  std_logic; 
        LOCK: out  std_logic);
end component;

-- parameterized module component instance
__ : pll_mux
    port map (CLKI=>__, CLKOP=>__, CLKOS=>__, CLKOS2=>__, LOCK=>__);