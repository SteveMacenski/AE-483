function varargout=discretize(policy,tspan,dt,t,x)

persistent u T step

% Hold onto control history. Only update every dt. If two outputs are
% asked for, spit back the control history and the related time vector. If
% only one output, give current control.
if strcmp(class(policy),'function_handle')
    if isempty(u)
        u_out=policy(t,x);
        T=tspan(1):dt:(tspan(2)+dt);
        u=zeros(length(u_out),length(T));
        u(:,1)=u_out;
        step=2;
    else
        if t>=T(step)
            u_out=policy(t,x);
            u(:,step)=u_out;
            step=step+1;
        else
            u_out=u(:,step-1);
        end
    end
end

if nargout==1
    varargout={u_out};
else
    varargout{1}=T(:,1:end-1);
    varargout{2}=u(:,1:end-1);
end

