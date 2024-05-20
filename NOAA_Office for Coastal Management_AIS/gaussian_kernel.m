function K = gaussian_kernel(x, y, sigma)
    K = exp(-norm(x-y)^2 / (2*sigma^2));
end
