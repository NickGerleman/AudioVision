function out = filter_exp(alpha, in)
    out = [];
    out(1) = in(1);
    for n = [2:length(in)]
        out(n) = alpha*in(n) + (1-alpha)*in(n-1);
    end