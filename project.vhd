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

-- Here I define the state necessary to carry out the computation
-- the first one is the IDLE state, which waits for the start signal
type state is (IDLE, MEM_WAIT, LOAD_XP, LOAD_YP, STORE_XP, STORE_YP, LOAD_MASK, STORE_MASK, CHECK_MASK, LOAD_XC, LOAD_YC, STORE_XC, STORE_YC, COMPUTE_DIST_PARTIAL, COMPUTE_DIST_TOTAL, CHECK_DIST, WRITE_RES, SIM_END);

signal P_X, P_Y, C_X, C_Y, MASK, O_MASK: std_logic_vector(7 downto 0);

signal centroid_curr, centroid_next: std_logic_vector(2 downto 0) := "000";

signal state_curr, state_next, return_curr, return_next: state := IDLE;

signal min_dist, temp_dist, temp_dist_x, temp_dist_y : std_logic_vector (8 downto 0) := "000000000";  

signal trigger: std_logic := '0';
signal discard : std_logic := '0';
    
    begin

-- Here I define the process to handle reset signals and clock steps
    process(i_clk, i_rst)
    begin
        if(i_rst='1') then
            state_curr <= IDLE;
        elsif(rising_edge(i_clk)) then
            if(state_next = state_curr) then trigger <= not(trigger);
            end if;
            state_curr <= state_next;
            return_curr <= return_next;
            centroid_curr <= centroid_next;
        end if;
    end process;
    
    RETURN_PROCESS : process(state_curr)
    begin
        case state_curr is  
            when LOAD_XP => return_next <= STORE_XP;
            when LOAD_YP => return_next <= STORE_YP;
            when LOAD_MASK => return_next <= STORE_MASK;
            when LOAD_XC => return_next <= STORE_XC;
            when LOAD_YC => return_next <= STORE_YC;
            when WRITE_RES => return_next <= SIM_END;
            when others => null;
        end case;
    end process;

    DATAUPDATE_PROCESS : process(state_curr, i_data)
    begin
        case(state_curr) is
            when STORE_XP => P_X <= i_data;
            when STORE_YP => P_Y <= i_data;
            when STORE_MASK => MASK <= i_data;
            when STORE_XC => C_X <= i_data;
            when STORE_YC => C_Y <= i_data;
            when others => null;
        end case;
    end process;

    CENTROID_PROCESS : process(state_curr)
    begin
        case(state_curr) is
            when CHECK_MASK => discard <= MASK(to_integer(unsigned(centroid_curr)));
                                if(MASK(to_integer(unsigned(centroid_curr))) = '0') then
                                     centroid_next <= std_logic_vector(unsigned(centroid_curr) + 1);
                                else centroid_next <= centroid_curr;
                                end if;
            when CHECK_DIST => discard <= '0';
                                centroid_next <= std_logic_vector(unsigned(centroid_curr) + 1);
            when others => centroid_next <= centroid_curr;
                            discard <= '0';
        end case;
    end process;
    
    NEXTSTATE_PROCESS : process(state_curr, i_start, trigger)
    begin
        -- Keep signals
        P_X <= P_X;
        P_Y <= P_Y;
        C_Y <= C_Y;
        C_X <= C_X;
        MASK <= MASK;
        O_MASK <= O_MASK;
        min_dist <= min_dist;
        temp_dist_x <= temp_dist_x;
        temp_dist_y <= temp_dist_y;
        
        case state_curr is       
            when IDLE =>
                        if(i_start = '1') then
                            state_next <= LOAD_XP;
                         else state_next <= IDLE;
                         end if;

            when LOAD_XP => state_next <= MEM_WAIT;    
            
            when STORE_XP => state_next <= LOAD_YP;                                      
                            
            when LOAD_YP => state_next <= MEM_WAIT;     
                            
            when STORE_YP => state_next <= LOAD_MASK;

            when LOAD_MASK => state_next <= MEM_WAIT;

            when STORE_MASK => state_next <= CHECK_MASK;
                               
            when CHECK_MASK => -- Check whether the current centroid is enabled or skip                                
                                if( centroid_curr = "111") then 
                                    state_next <= WRITE_RES;
                                else 
                                    if(MASK(to_integer(unsigned(centroid_curr))) = '1') then
                                        state_next <= LOAD_XC;
                                    elsif(MASK(to_integer(unsigned(centroid_curr))) = '0') then
                                        -- The centroid is not enabled, increment the centroid and jump to the next one                                    
                                        state_next <= CHECK_MASK;                                    
                                    end if;
                                end if;
                                
            -- Load and store centroids position
            when LOAD_XC => state_next <= MEM_WAIT;
            when LOAD_YC => state_next <= MEM_WAIT;
            when STORE_XC => state_next <= LOAD_YC;
            when STORE_YC => state_next <= COMPUTE_DIST_PARTIAL;
                                
            -- Compute and compare the distance
            when COMPUTE_DIST_PARTIAL => 
                                 temp_dist_x <= std_logic_vector(abs(signed(unsigned('0'&P_X) - unsigned('0'&C_X))));
                                 temp_dist_y <= std_logic_vector(abs(signed(unsigned('0'&P_Y) - unsigned('0'&C_Y))));
                                 state_next <= COMPUTE_DIST_TOTAL;
                                 
            when COMPUTE_DIST_TOTAL => temp_dist <= std_logic_vector(unsigned(temp_dist_x) + unsigned(temp_dist_y));
                                       state_next <= CHECK_DIST;
                     
                     -- The distance is not initialized -> store & save
            when CHECK_DIST =>  if(min_dist = "000000000" or temp_dist < min_dist) then 
                                    O_MASK <= "00000000"; -- clear the output mask;                                    
                                    O_MASK(to_integer(unsigned(centroid_curr))) <= '1';
                                    min_dist <= temp_dist;
                                elsif(min_dist = temp_dist) then
                                    O_MASK(to_integer(unsigned(centroid_curr))) <= '1'; -- just add the bit to the mask;
                                end if;
                                state_next <= CHECK_MASK;          
            
            -- Here I handle the logic of waiting for the memory to output the data 
            when MEM_WAIT => state_next <= return_curr;
            
            when WRITE_RES => state_next <= MEM_WAIT;
            
            when SIM_END => if(i_start = '1') then 
                                state_next <= SIM_END;
                            elsif(i_start = '0') then 
                                state_next <= IDLE;
                            end if;
            when others => null;                
        end case;             
    end process;
    
    OUT_PROCESS : process(state_curr)
    begin
    case(state_curr) is
        when LOAD_XP => 
                    o_done <= '0';
                    o_address <= "0000000000010001";
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= "00000000";
        when LOAD_YP => 
                    o_done <= '0';
                    o_address <= "0000000000010010";
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= "00000000";
        when LOAD_MASK => 
                    o_done <= '0';
                    o_address <= "0000000000000000";
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= "00000000";
        when LOAD_XC => 
                    o_done <= '0';
                    o_address <= std_logic_vector(unsigned("000000000000"&centroid_curr&"0") + 1);
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= "00000000";
        when LOAD_YC => 
                    o_done <= '0';
                    o_address <= std_logic_vector(unsigned("000000000000"&centroid_curr&"0") + 2);
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= "00000000";
        when WRITE_RES => 
                    o_done <= '0';
                    o_address <= "0000000000010011";
                    o_en <= '1';
                    o_we <= '1';
                    o_data <= O_MASK;
        when SIM_END => 
                    if(i_start = '1') then
                        o_done <= '1';
                    else o_done <= '0';
                    end if;
                    o_address <= "0000000000000000";
                    o_en <= '0';
                    o_we <= '0';
                    o_data <= "00000000";
        when others => 
                    o_done <= '0';
                    o_address <= "0000000000000000";
                    o_en <= '0';
                    o_we <= '0';
                    o_data <= "00000000";                       
    end case;
    end process;

end Behavioral;
