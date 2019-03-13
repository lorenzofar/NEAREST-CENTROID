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

type state is (IDLE, LOAD_XP, LOAD_YP, STORE_XP, STORE_YP, LOAD_MASK, STORE_MASK, CHECK_MASK, LOAD_XC, LOAD_YC, STORE_XC, STORE_YC, COMPUTE_DIST, CHECK_DIST, WRITE_RES, SIM_END);

-- These signals are used to store the coordinates of the centroid and the main point, as well as the reference mask and output mask
signal px_curr, px_next, py_curr, py_next, cx_curr, cx_next, cy_curr, cy_next, MASK, omask_curr, omask_next: unsigned(7 downto 0) := (others => '0');

-- These signals are used to keep track of the current centroid, in order to stop computation and to access vectors by index
signal centroid_curr, centroid_next: std_logic_vector(2 downto 0) := (others => '0');

-- These signals are used to manage the state the FSM is into
signal state_curr, state_next : state := IDLE;

-- These signals are used to store the current minimum distance and the temporary computed distance
signal dmin_curr, dmin_next, dtemp: unsigned(8 downto 0) := (others => '1');  

-- These signals are instead used to manage the done output once the simulation has ended
signal done_curr, done_next : std_logic := '0';
    
    begin

    -- This process handles clock and reset signals 
    -- When we are on the rising edge of the i_clk signal we update the current value of the several signals
    -- When instead we are on the rising edge of the i_rst signal we put the system back in the initial state, ready to start a new simulation
    CLOCK_PROCESS : process(i_clk, i_rst)
    begin
        if(i_rst = '1') then
            centroid_curr <= (others => '0');
            omask_curr <= (others => '0');
            done_curr <= '0'; 
            dmin_curr <= (others => '0');
            state_curr <= IDLE;
        elsif(rising_edge(i_clk)) then            
            centroid_curr <= centroid_next;
            omask_curr <= omask_next;
            done_curr <= done_next;
            dmin_curr <= dmin_next;
            state_curr <= state_next;
            px_curr <= px_next;
            py_curr <= py_next;
            cx_curr <= cx_next;
            cy_curr <= cy_next;
        end if;
    end process;

    -- This process handles the data received from the memory
    -- According to the state the FSM is into, signals are updated
    -- Before we check for the state, the current value of the signals is assigned to the next one, in order to keep the information
    -- Then, after determining the state, we update just one signal
    -- If we are in a state that does not need to read from memory, we do nothing
    DATAUPDATE_PROCESS : process(i_data) 
    begin
        px_next <= px_curr;
        py_next <= py_curr;
        cx_next <= cx_curr;
        cy_next <= cy_curr;
        MASK <= MASK;
        case(state_curr) is
            when STORE_XP => 
                px_next <= unsigned(i_data);
            when STORE_YP =>
                py_next <= unsigned(i_data);
            when STORE_XC => 
                cx_next <= unsigned(i_data);
            when STORE_YC => 
                cy_next <= unsigned(i_data);
            when STORE_MASK => 
                MASK <= unsigned(i_data);
            when others => null;
        end case;
    end process;
    
    -- This process is responsible of managing the FSM's states and of carrying out the computation
    -- It is sensible to the i_start signal to start the simulation, to the i_rst signal to reset the system,
    -- to the state_curr signal to perform the specific operations of each state and to the centroid_curr signal to trigger a new cycle when the current centroid changes
    NEXTSTATE_PROCESS : process(state_curr, i_start, i_rst, centroid_curr)
    begin
        if(i_rst = '1') then
            -- Here we set all the next values of signals to 0, in order to reset the system and start again
            centroid_next <= (others => '0');
            done_next <= '0';
            omask_next <= (others => '0');
            dmin_next <= (others => '0');      
            state_next <= IDLE;
        else 
            -- Here we assign the current value of signals to the next one, in order to keep information
            omask_next <= omask_curr;
            dmin_next <= dmin_curr;       
            centroid_next <= centroid_curr;
            done_next <= '0';
            
            -- Then we determine in which state the FSM is 
            case state_curr is       
                when IDLE =>
                    if(i_start = '1') then
                        state_next <= LOAD_XP; -- The system received the start signal, we start the simulation
                    else state_next <= IDLE; -- We continue to wait for the i_start signal to be 1
                    end if;
    
                when LOAD_XP => 
                    state_next <= STORE_XP;                   
                when STORE_XP => 
                    state_next <= LOAD_YP;   
                when LOAD_YP => 
                    state_next <= STORE_YP;   
                when STORE_YP => 
                    state_next <= LOAD_MASK;
                when LOAD_MASK => 
                    state_next <= STORE_MASK;
                when STORE_MASK => 
                    state_next <= CHECK_MASK;
                             
                when CHECK_MASK =>
                    if( centroid_curr = "111" and MASK(to_integer(unsigned(centroid_curr))) = '0') then 
                        state_next <= WRITE_RES;
                    else 
                        if(MASK(to_integer(unsigned(centroid_curr))) = '1') then                                        
                            centroid_next <= centroid_curr;
                            state_next <= LOAD_XC;
                        elsif(MASK(to_integer(unsigned(centroid_curr))) = '0') then                                  
                            centroid_next <= std_logic_vector(unsigned(centroid_curr) + 1);
                            state_next <= CHECK_MASK;                                    
                        end if;
                    end if;
                                    
                -- Load and store centroids position
                when LOAD_XC => 
                    state_next <= STORE_XC;
                when LOAD_YC => 
                    state_next <= STORE_YC;
                when STORE_XC => 
                    state_next <= LOAD_YC;
                when STORE_YC => 
                    state_next <= COMPUTE_DIST;
                                                                    
                when COMPUTE_DIST =>
                    dtemp <= 
                    to_unsigned(
                        abs(to_integer(px_curr) - to_integer(cx_curr))
                        + 
                        abs(to_integer(py_curr) - to_integer(cy_curr)),
                        dtemp'length
                    );
                    state_next <= CHECK_DIST;
                         
                -- The distance is not initialized -> store & save
                when CHECK_DIST =>  
                    if(dmin_curr = "111111111" or dtemp < dmin_curr) then 
                        omask_next <= (others => '0'); -- clear the output mask;                                    
                        omask_next(to_integer(unsigned(centroid_curr))) <= '1';
                        dmin_next <= dtemp;
                    elsif(dmin_curr = dtemp) then
                        omask_next(to_integer(unsigned(centroid_curr))) <= '1';
                    else 
                        omask_next(to_integer(unsigned(centroid_curr))) <= '0';
                    end if;
                    
                    -- Check whether I arrived at last step
                    if(centroid_curr = "111") then
                        centroid_next <= centroid_curr;
                        state_next <= WRITE_RES;
                    else                                                                                
                        centroid_next <= std_logic_vector(unsigned(centroid_curr) + 1);                                    
                        state_next <= CHECK_MASK;
                    end if;                                     

                when WRITE_RES =>   
                    done_next <= '1';
                    state_next <= SIM_END;
                
                when SIM_END => 
                    if(i_start = '1') then
                        state_next <= SIM_END;
                    else 
                        done_next <= '0';
                        state_next <= IDLE;
                    end if;

                when others => null;                
            end case; 
        end if;            
    end process;
    
    OUT_PROCESS : process(state_curr)
    begin
        o_done <= done_curr;
        case(state_curr) is
            when LOAD_XP => 
                        o_address <= std_logic_vector(to_unsigned(17, o_adddress'length));
                        o_en <= '1';
                        o_we <= '0';
                        o_data <= (others => '0');
            when LOAD_YP => 
                        o_address <= std_logic_vector(to_unsigned(18, o_adddress'length));
                        o_en <= '1';
                        o_we <= '0';
                        o_data <= (others => '0');
            when LOAD_MASK => 
                        o_address <= (others => '0');
                        o_en <= '1';
                        o_we <= '0';
                        o_data <= (others => '0');
            when LOAD_XC => 
                        o_address <= std_logic_vector(unsigned("000000000000"&centroid_curr&"0") + 1);
                        o_en <= '1';
                        o_we <= '0';
                        o_data <= (others => '0');
            when LOAD_YC => 
                        o_address <= std_logic_vector(unsigned("000000000000"&centroid_curr&"0") + 2);
                        o_en <= '1';
                        o_we <= '0';
                        o_data <= (others => '0');
            when WRITE_RES => 
                        o_address <= std_logic_vector(to_unsigned(19, o_address'length));
                        o_en <= '1';
                        o_we <= '1';
                        o_data <= std_logic_vector(omask_curr);
            when SIM_END => 
                        o_address <= (others => '0');
                        o_en <= '0';
                        o_we <= '0';
                        o_data <= (others => '0');
            when others => 
                        o_address <= (others => '0');
                        o_en <= '0';
                        o_we <= '0';
                        o_data <= (others => '0');
        end case;
    end process;

end Behavioral;