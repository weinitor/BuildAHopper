function MQ = fivebarIK(Q , param)
    Q(Q(:,1)>param.r)=param.r;
    MQ = acos((-param.l1^2+param.l2^2-Q(:,1).^2)./(2*param.l1.*Q(:,1)));
end

