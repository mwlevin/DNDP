class Autograde:

    def __init__(self):
        self.total = 0
        self.points = 0
        self.overall_points = 0
        self.overall_total = 0
        self.printAutogradeHeader()

    def test(self, passed):
        self.total += 1

        if passed:
            self.points += 1

    def flush(self, label):
        msg = "Testing " + label + ": " + str(self.points) + " / "
        pct = round(self.points * 100.0 / self.total)
        msg += str(self.total) + " (" + str(pct) + "%)"
        
        print(msg)

        self.overall_points += self.points
        self.overall_total += self.total

        self.total = 0
        self.points = 0

    def end(self):
        
        pct = round(self.overall_points * 100.0 / self.overall_total)
        
        msg = "\nTotal: " + str(self.overall_points) + " / "
        msg += str(self.overall_total) + " (" + str(pct) + "%)"
        print(msg)

        self.printAutogradeFooter()

    def printAutogradeHeader(self):
        print("\n\n********************")
        print("*    Autograde     *")
        print("********************")

    def printAutogradeFooter(self):
        print("\n********************")
        print("*  End Autograde   *")
        print("********************")

    