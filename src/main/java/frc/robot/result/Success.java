package frc.robot.result;

import java.util.function.Function;

/** 
 * Represents a successful operation. 
 */
public record Success<R, E>(R value) implements Result<R, E> {
    @Override
    public boolean isSuccess() {
        return true;
    }

    @Override
    public boolean isFailure() {
        return false;
    }

    @Override
    public R getValue() {
        return value;
    }

    @Override
    public E getError() {
        throw new IllegalStateException("Success state cannot represent errors.");
    }

    @Override
    public <R1> Result<R1, E> map(Function<? super R, ? extends R1> mapper) {
        return new Success<>(mapper.apply(value));
    }

    @Override
    public <R1> Result<R1, E> flatMap(Function<? super R, Result<R1, E>> mapper) {
        return mapper.apply(value);
    }
    
    @Override
    public <E1> Result<R, E1> mapError(Function<? super E, ? extends E1> mapper) {
        return new Success<>(value);
    }

    @Override
    public <R1> R1 fold(Function<? super R, ? extends R1> onSuccess, Function<? super E, ? extends R1> onFailure) {
        return onSuccess.apply(value);
    }
}
