package frc.lib;

import com.google.auto.service.AutoService;
import com.google.errorprone.BugPattern;
import com.google.errorprone.matchers.Description;
import com.sun.source.tree.ExpressionTree;
import com.sun.source.tree.MethodInvocationTree;
import com.sun.source.tree.NewClassTree;
import com.google.errorprone.util.ASTHelpers;
import com.google.errorprone.VisitorState;
import com.google.errorprone.bugpatterns.BugChecker;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.Type.ClassType;
import java.util.List;

import static com.google.errorprone.BugPattern.SeverityLevel.ERROR;

@AutoService(BugChecker.class)
@BugPattern(
    name = "NoSubtypeAllowed",
    summary = "Disallows passing subclasses to parameters annotated with @NoSubtypeAllowed",
    severity = ERROR)
public class NoSubtypeAllowedChecker extends BugChecker
    implements BugChecker.NewClassTreeMatcher,
    BugChecker.MethodInvocationTreeMatcher {

    private Description checkArgumentsForSubtypeViolation(
        List<Symbol.VarSymbol> parameters,
        List<? extends ExpressionTree> arguments,
        VisitorState state)
    {

        for (int i = 0; i < Math.min(parameters.size(), arguments.size()); i++) {
            Symbol.VarSymbol param = parameters.get(i);
            ExpressionTree argument = arguments.get(i);

            if (ASTHelpers.hasAnnotation(param, "frc.lib.annotations.NoSubtypeAllowed", state)) {
                Type paramType = param.type;
                Type argType = ASTHelpers.getType(argument);

                if (argType == null) {
                    return Description.NO_MATCH;
                }

                boolean exactMatch = argType.tsym.equals(paramType.tsym);

                boolean directInterfaceImplementation = false;
                if (!exactMatch && paramType.tsym.isInterface() && argType instanceof ClassType) {
                    ClassType classType = (ClassType) argType;
                    List<Type> interfaces = classType.interfaces_field;
                    for (Type iface : interfaces) {
                        if (iface.tsym.equals(paramType.tsym)) {
                            directInterfaceImplementation = true;
                            break;
                        }
                    }
                }

                if (!exactMatch && !directInterfaceImplementation) {
                    return buildDescription(argument)
                        .setMessage("Only the exact type or direct implementation of interface "
                            + paramType.tsym.getQualifiedName() + " is allowed.")
                        .build();
                }
            }
        }

        return Description.NO_MATCH;
    }


    @Override
    public Description matchNewClass(NewClassTree newClassTree, VisitorState state)
    {
        Symbol.MethodSymbol constructorSymbol =
            (Symbol.MethodSymbol) ASTHelpers.getSymbol(newClassTree);
        if (constructorSymbol == null || !constructorSymbol.isConstructor()) {
            return Description.NO_MATCH;
        }

        List<Symbol.VarSymbol> parameters = constructorSymbol.getParameters();
        List<? extends ExpressionTree> arguments = newClassTree.getArguments();

        return checkArgumentsForSubtypeViolation(parameters, arguments, state);
    }

    @Override
    public Description matchMethodInvocation(MethodInvocationTree methodInvocationTree,
        VisitorState state)
    {
        Symbol.MethodSymbol methodSymbol =
            (Symbol.MethodSymbol) ASTHelpers.getSymbol(methodInvocationTree);
        if (methodSymbol == null) {
            return Description.NO_MATCH;
        }

        List<Symbol.VarSymbol> parameters = methodSymbol.getParameters();
        List<? extends ExpressionTree> arguments = methodInvocationTree.getArguments();

        return checkArgumentsForSubtypeViolation(parameters, arguments, state);
    }
}
