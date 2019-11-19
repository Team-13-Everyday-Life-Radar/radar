function [peak_idx, peak_val] = f_search_peak(fft_spectrum, search_length, threshold, max_target_count, Min_Distance, Max_Distance, DistPerBin)
    
    peak_cnt = 0;
    
    peak_val = 0;
    
    peak_idx = [];
    
    for n = 3:(search_length - 3)
        
        fp_bin  = n;
		fl_bin  = fp_bin - 1;
        fl2_bin = fp_bin - 2;
		fr_bin  = fp_bin + 1;
        fr2_bin = fp_bin + 2;

		fp  = fft_spectrum(fp_bin);
		fl  = fft_spectrum(fl_bin);
        fl2 = fft_spectrum(fl2_bin);
		fr  = fft_spectrum(fr_bin);
        fr2 = fft_spectrum(fr2_bin);
                        
        if(fp >= threshold && fp >= fl2 && fp >= fl && fp > fr && fp > fr2)
            
            curr_range = (fp_bin - 1) * DistPerBin;

            if (curr_range >= Min_Distance && curr_range <= Max_Distance)
                
                peak_cnt = peak_cnt + 1;

                peak_val(peak_cnt) = fp;

                peak_idx(peak_cnt) = fp_bin;

                if (peak_cnt >= max_target_count)
                   break; 
                end                
            end                     
        end                
    end
end
