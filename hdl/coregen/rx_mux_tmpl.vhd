-- VHDL module instantiation generated by SCUBA Diamond (64-bit) 3.6.0.83.4
-- Module  Version: 5.8
-- Thu Oct 29 19:51:17 2015

-- parameterized module component declaration
component rx_mux
    port (alignwd: in  std_logic; clk: in  std_logic; 
        clk_s: in  std_logic; del: in  std_logic_vector(4 downto 0); 
        init: in  std_logic; reset: in  std_logic; 
        rx_ready: out  std_logic; sclk: out  std_logic; 
        datain: in  std_logic_vector(0 downto 0); 
        q: out  std_logic_vector(7 downto 0));
end component;

-- parameterized module component instance
__ : rx_mux
    port map (alignwd=>__, clk=>__, clk_s=>__, del(4 downto 0)=>__, init=>__, 
        reset=>__, rx_ready=>__, sclk=>__, datain(0 downto 0)=>__, q(7 downto 0)=>__);
