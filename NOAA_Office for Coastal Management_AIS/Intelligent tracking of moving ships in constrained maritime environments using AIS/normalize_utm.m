function [x_norm, y_norm] = normalize_utm(x, y)
    % Calcola la media e la deviazione standard delle coordinate UTM
    x_mean = mean(x);
    y_mean = mean(y);
    x_std = std(x);
    y_std = std(y);
    
    % Normalizza le coordinate UTM
    x_norm = (x - x_mean) / x_std;
    y_norm = (y - y_mean) / y_std;
end