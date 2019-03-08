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
type state is (IDLE, LOAD_XP, LOAD_YP, STORE_XP, STORE_YP, LOAD_MASK, STORE_MASK, CHECK_MASK, LOAD_XC, LOAD_YC, STORE_XC, STORE_YC, COMPUTE_DIST, CHECK_DIST, WRITE_RES, SIM_END);

signal px_curr, px_next, py_curr, py_next, cx_curr, cx_next, cy_curr, cy_next, MASK, omask_curr, omask_next: std_logic_vector(7 downto 0) := (others => '0');

signal centroid_curr, centroid_next: std_logic_vector(2 downto 0) := (others => '0');

signal state_curr, state_next : state := IDLE;

signal dmin_curr, dmin_next, dtemp: std_logic_vector (8 downto 0) := (others => '0');  

signal done_curr, done_next : std_logic := '0';
    
    begin

-- Here I define the process to handle reset signals and clock steps
    process(i_clk, i_rst)
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

    DATAUPDATE_PROCESS : process(i_data) 
    begin
        px_next <= px_curr;
        py_next <= py_curr;
        cx_next <= cx_curr;
        cy_next <= cy_curr;
        MASK <= MASK;
        case(state_curr) is
            when STORE_XP => px_next <= i_data;
            when STORE_YP => py_next <= i_data;
            when STORE_XC => cx_next <= i_data;
            when STORE_YC => cy_next <= i_data;
            when STORE_MASK => MASK <= i_data;
            when others => null;
        end case;
    end process;
    
    NEXTSTATE_PROCESS : process(state_curr, i_start, i_rst, centroid_curr)
    begin
        if(i_rst = '1') then
            centroid_next <= (others => '0');
            done_next <= '0';
            omask_next <= (others => '0');
            dmin_next <= (others => '0');      
            state_next <= IDLE;
        else 

            omask_next <= omask_curr;
            dmin_next <= dmin_curr;       
            centroid_next <= centroid_curr;
            done_next <= '0';
            
            case state_curr is       
                when IDLE =>
                            if(i_start = '1') then
                                state_next <= LOAD_XP;
                            else state_next <= IDLE;
                            end if;
    
                when LOAD_XP => state_next <= STORE_XP;                   
                when STORE_XP => state_next <= LOAD_YP;   
                when LOAD_YP => state_next <= STORE_YP;   
                when STORE_YP => state_next <= LOAD_MASK;
                when LOAD_MASK => state_next <= STORE_MASK;
                when STORE_MASK => state_next <= CHECK_MASK;
                                   
                when CHECK_MASK => -- Check whether the current centroid is enabled or skip                                
                                    if( centroid_curr = "111") then 
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
                when LOAD_XC => state_next <= STORE_XC;
                when LOAD_YC => state_next <= STORE_YC;
                when STORE_XC => state_next <= LOAD_YC;
                when STORE_YC => state_next <= COMPUTE_DIST;
                                                                    
                when COMPUTE_DIST => dtemp <= std_logic_vector(abs(signed(unsigned('0'&px_curr) - unsigned('0'&cx_curr))) + abs(signed(unsigned('0'&py_curr) - unsigned('0'&cy_curr))));
                                     state_next <= CHECK_DIST;
                         
                         -- The distance is not initialized -> store & save
                when CHECK_DIST =>  if(dmin_curr = "000000000" or dtemp < dmin_curr) then 
                                        omask_next <= (others => '0'); -- clear the output mask;                                    
                                        omask_next(to_integer(unsigned(centroid_curr))) <= '1';
                                        dmin_next <= dtemp;
                                    elsif(dmin_curr = dtemp) then
                                        omask_next(to_integer(unsigned(centroid_curr))) <= '1';
                                    else 
                                        omask_next(to_integer(unsigned(centroid_curr))) <= '0';
                                    end if;
                                    centroid_next <= std_logic_vector(unsigned(centroid_curr) + 1);
                                    state_next <= CHECK_MASK;          
                            
                when WRITE_RES =>   done_next <= '1';
                                    state_next <= SIM_END;
                
                when SIM_END => if(i_start = '1') then
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
                    o_address <= "0000000000010001";
                    o_en <= '1';
                    o_we <= '0';
                    o_data <= (others => '0');
        when LOAD_YP => 
                    o_address <= "0000000000010010";
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
                    o_address <= "0000000000010011";
                    o_en <= '1';
                    o_we <= '1';
                    o_data <= omask_curr;
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
