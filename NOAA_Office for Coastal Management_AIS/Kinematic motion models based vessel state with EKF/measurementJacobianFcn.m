function H = measurementJacobianFcn(~)
    % Il modello di misura è lineare e restituisce i primi quattro stati
    H = [eye(4), zeros(4, 2)];
end