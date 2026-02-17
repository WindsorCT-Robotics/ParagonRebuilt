package frc.robot.result;

import java.util.function.Function;

/**
 * Represents an operation that may succeed or fail.
 * @param R The result returned from operation success.
 * @param E The error returned from operation failure.
 */
public sealed interface Result<R, E> permits Success, Failure {
    public boolean isSuccess();
    public boolean isFailure();

    public R getValue();
    public E getError();

    public <R1> Result<R1, E> map(Function<? super R, ? extends R1> mapper);
    public <R1> Result<R1, E> flatMap(Function<? super R, Result<R1, E>> mapper);
    public <E1> Result<R, E1> mapError(Function<? super E, ? extends E1> mapper);
    public <R1> R1 fold(
        Function<? super R, ? extends R1> onSuccess
        , Function<? super E, ? extends R1> onFailure);
}