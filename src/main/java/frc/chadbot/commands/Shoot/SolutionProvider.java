package frc.chadbot.commands.Shoot;

// Used to hook in a command or other object that tells us we can shoot
// override this based on your targeting system
public interface SolutionProvider {
        default public boolean isOnTarget() { return true;}
}
