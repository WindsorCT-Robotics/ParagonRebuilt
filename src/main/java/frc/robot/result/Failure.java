package frc.robot.result;

import java.util.function.Function;

/**
 * Represents a failed operation.
 */
public record Failure<R, E>(E err) implements Result<R, E> {

    @Override
    public boolean isSuccess() {
        return false;
    }

    @Override
    public boolean isFailure() {
        return true;
    }

    @Override
    public R getValue() {
        throw new IllegalStateException("Failure state cannot represent values.");
    }

    @Override
    public E getError() {
        return err;
    }

    @Override
    public <R1> Result<R1, E> map(Function<? super R, ? extends R1> mapper) {
        return new Failure<>(err);
    }

    @Override
    public <R1> Result<R1, E> flatMap(Function<? super R, Result<R1, E>> mapper) {
        return new Failure<>(err);
    }
    
    @Override
    public <E1> Result<R, E1> mapError(Function<? super E, ? extends E1> mapper) {
        return new Failure<>(mapper.apply(err));
    }
    
    @Override
    public <R1> R1 fold(Function<? super R, ? extends R1> onSuccess, Function<? super E, ? extends R1> onFailure) {
        return onFailure.apply(err);
    }
}
