----------------------------------------------------------------------------------
-- Company: 
-- Engineer: Lorenzo Farinelli - 866236
-- 
-- Create Date: 04/21/2019 02:04:39 PM
-- Design Name: Progetto di reti logiche
-- Module Name: project_reti_logiche- Behavioral
-- Project Name: 
-- Target Devices: Artix-7 xc7a200tfbg484-1
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity project_reti_logiche is
    Port ( 
           i_start : in STD_LOGIC;
           i_rst : in STD_LOGIC;
           i_clk : in STD_LOGIC;
           i_data : in STD_LOGIC_VECTOR (7 downto 0 );
           o_address: out STD_LOGIC_VECTOR (15 downto 0);
           o_done: out STD_LOGIC;
           o_en: out STD_LOGIC;
           o_we: out STD_LOGIC;
           o_data: out STD_LOGIC_VECTOR (7 downto 0));
end project_reti_logiche;

architecture Behavioral of project_reti_logiche is  

type state is (IDLE, STORE_XP, STORE_YP, LOAD_MASK, STORE_MASK, CHECK_MASK, STORE_XC, STORE_YC, CHECK_DIST, WRITE_RES, SIM_END);

-- These signals are used to store the coordinates of the centroid and the main point, as well as the reference mask and output mask
signal px, py, cx, cy, mask, omask: unsigned(7 downto 0) := (others => '0');

-- This signal is used to keep track of the current centroid, in order to stop computation and to access mask bits
signal centroid: unsigned(2 downto 0) := (others => '0');

-- These signals are used to manage the state the FSM is into
signal state_curr, state_next : state := IDLE;

-- These signals are used to store the current minimum distance and the temporary computed distance
signal dmin, dtemp: unsigned(8 downto 0) := (others => '1');  

-- This signal is instead used to manage the done output notifying computation end
signal done: std_logic := '0'; 
        
begin    
    
    -- This process handles the data received from the memory
    -- According to the state the FSM is into, signals are updated
    -- If we are in a state that does not need to read from memory, we do nothing
    DATA_PROCESS : process(i_clk, i_rst, i_data)
    begin
        if(i_rst = '1') then  
            mask <= (others => '0');   
            px <= (others => '0');
            py <= (others => '0');
            cx <= (others => '0');
            cy <= (others => '0');   
        else
            mask <= mask;
            px <= px;
            py <= py;
            cx <= cx;
            cy <= cy;          
            if(falling_edge(i_clk)) then
                case(state_curr) is
                    when STORE_MASK => 
                        mask <= unsigned(i_data);
                    when STORE_XP => 
                        px <= unsigned(i_data);
                    when STORE_YP => 
                        py <= unsigned(i_data);
                    when STORE_XC =>
                        cx <= unsigned(i_data);
                    when STORE_YC => 
                        cy <= unsigned(i_data);
                    when others => 
                        null;
                end case; 
            else
                null;               
            end if; 
        end if;           
    end process;
    
    -- This process is responsible of managing the output mask
    -- It updates the signal on the clock's rising edge
    -- When the FSM is in the CHECK_DIST state it checks whether 
    -- the current centroid has a new minimum distance or not 
    -- and updates the output mask accordingly
    -- In the other states it just sets the current value
    OMASK_PROCESS : process(i_clk, i_rst, state_curr)
    begin
        if(i_rst = '1') then
            omask <= (others => '0');
        else 
            if(falling_edge(i_clk) and state_curr = CHECK_DIST) then
                if(dmin = "111111111" or dtemp < dmin) then 
                    omask <= (others => '0'); -- clear the output mask;                                    
                    omask(to_integer(unsigned(centroid))) <= '1';
                elsif(dmin = dtemp) then
                    omask(to_integer(unsigned(centroid))) <= '1';
                else 
                    omask(to_integer(unsigned(centroid))) <= '0';
                end if;
            else 
                omask <= omask;
            end if;
        end if;
    end process;
    
    -- This process computes the Manhattan distance
    -- of the two points as soon as their coordinates change
    -- It is not bounded to any particular FSM state
    DIST_PROCESS: process(px, py, cx, cy)
    begin
        dtemp <= 
        to_unsigned(
            abs(to_integer(px) - to_integer(cx))
            + 
            abs(to_integer(py) - to_integer(cy)),
            dtemp'length
        );            
    end process;
    
    -- This state handles the signal storing the minimum distance
    -- It updates on the clock's falling edge, to avoid interferences with OMASK_PROCESS
    DMIN_PROCESS : process(i_clk, i_rst, state_curr)
    begin
        if(i_rst = '1') then
            dmin <= (others => '1');
        else
            if(rising_edge(i_clk) and state_curr = CHECK_DIST and (dmin = "111111111" or dtemp < dmin)) then
                dmin <= dtemp;
            else 
                dmin <= dmin;
            end if;
        end if;
    end process;
    
    -- This process updates the counter of the current centroid
    -- When the FSM is in a state where it does not need to increase the counter, 
    -- it just keeps the current value
    CENTROID_PROCESS: process(i_clk, i_rst, state_curr)
    begin
        if(i_rst = '1') then
            centroid <= (others => '0');
        else 
            if(rising_edge(i_clk)) then
                case(state_curr) is
                    when CHECK_MASK => 
                        if(centroid = to_unsigned(7, centroid'length) or mask(to_integer(centroid)) = '1') then
                            centroid <= centroid;
                        else
                            centroid <= centroid + 1;
                        end if;
                    when CHECK_DIST => 
                        if(not(centroid = to_unsigned(7, centroid'length))) then
                            centroid <= centroid + 1;
                        else 
                            centroid <= centroid;
                        end if;
                    when others => 
                        centroid <= centroid;
                end case;
            else
                centroid <= centroid;
            end if;            
        end if;
    end process;
    
    -- This process updates the signal used to notify the end of the computation
    -- When the FSM is in its final state (SIM_END), its value is the same of the clock
    -- and goes to 0 when the clock is put to 0, as required by the specification
    DONE_PROCESS : process(i_clk, i_rst, state_curr)        
    begin
        if(i_rst = '1') then
            done <= '0';
        else
            if(rising_edge(i_clk)) then
                case(state_curr) is
                    when WRITE_RES => 
                        done <= '1';
                    when SIM_END => 
                        done <= i_start;
                    when others => 
                        done <= done;
                end case;
            else
                done <= done;
            end if;
        end if;
    end process;
    
    -- This process handles clock and reset signals 
    -- When we are on the rising edge of the i_clk signal we update the curren state of the FSM
    -- When the i_rst signal is set to 1 we put the FSM back in the IDLE state
    CLOCK_PROCESS : process(i_clk, i_rst)
    begin
        if(rising_edge(i_clk)) then
            -- Here we always update the coordinates, since we do not care to reset them
            if(i_rst = '1') then
                -- Here we have to reset the machine state
                -- We reset only critical signals
                state_curr <= IDLE;
            else             
                state_curr <= state_next;
            end if;
        end if;
    end process;
    
    -- This process is responsible of managing the FSM's states by setting the next states
    -- It is sensible to the i_start signal to start the simulation, to the i_rst signal to reset the system,
    NEXTSTATE_PROCESS : process(i_clk, i_start, i_rst)
    begin
        if(i_rst = '1') then
            state_next <= IDLE;
        else     
            state_next <= state_curr;
            
            case state_curr is       
                when IDLE =>                
                    if(i_start = '1') then
                        state_next <= LOAD_MASK; -- The system received the start signal, we start the simulation
                    else 
                        state_next <= IDLE; -- We continue to wait for the i_start signal to be 1
                    end if;
                        
                when LOAD_MASK => 
                    state_next <= STORE_MASK;
                 when STORE_MASK => 
                    state_next <= STORE_XP;
                when STORE_XP => 
                    state_next <= STORE_YP;                   
                when STORE_YP => 
                    state_next <= CHECK_MASK;                
                            
                when CHECK_MASK =>
                    if(centroid = to_unsigned(7, centroid'length) and mask(to_integer(centroid)) = '0') then 
                        state_next <= WRITE_RES;
                    else 
                        if(mask(to_integer(centroid)) = '1') then    
                            state_next <= STORE_XC;
                        else                              
                            state_next <= CHECK_MASK;                                    
                        end if;
                    end if;
                                    
                when STORE_XC => 
                    state_next <= STORE_YC;
                when STORE_YC => 
                    state_next <= CHECK_DIST;
                        
                when CHECK_DIST =>  
                    if(centroid = to_unsigned(7, centroid'length)) then
                        state_next <= WRITE_RES;
                    else                                                                                         
                        state_next <= CHECK_MASK;
                    end if;                                     

                when WRITE_RES =>   
                    state_next <= SIM_END;

                when SIM_END => 
                    if(i_start = '1') then
                        state_next <= SIM_END;
                    else 
                        state_next <= IDLE;
                    end if;         
            end case; 
        end if;      
    end process;
    
    -- This process updates the signal coming out of the FSM
    -- It is hence responsible of setting the address from which require data,
    -- of setting the memory enable signals to read/write data and 
    -- of setting the o_done signal with the value of done, to notify the end of the computation 
    OUT_PROCESS : process(state_curr, centroid)
    begin
        o_done <= done;
        o_we <= '0';
        o_data <= (others => '0');
        case(state_curr) is 
            when LOAD_MASK => 
                o_address <= (others => '0');
                o_en <= '1';
            when STORE_MASK => 
                o_address <= std_logic_vector(to_unsigned(17, o_address'length));
                o_en <= '1';
            when STORE_XP => 
                o_address <= std_logic_vector(to_unsigned(18, o_address'length));
                o_en <= '1';
           
            when CHECK_MASK => 
                o_address <= std_logic_vector(unsigned("000000000000"&centroid&"0") + 1);
                o_en <= '1';
            when STORE_XC => 
                o_address <= std_logic_vector(unsigned("000000000000"&centroid&"0") + 2);
                o_en <= '1';                        
            when WRITE_RES => 
                o_address <= std_logic_vector(to_unsigned(19, o_address'length));
                o_en <= '1';
                o_we <= '1';
                o_data <= std_logic_vector(omask);
            when others => 
                o_address <= (others => '0');
                o_en <= '0';
        end case;
    end process;

end Behavioral;