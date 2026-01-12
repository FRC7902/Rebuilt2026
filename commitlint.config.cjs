module.exports = {
  parserPreset: {
    parserOpts: {
      // Accept:
      //   ABCD-12: message
      //   ABCD-123: message
      //   HTFX: message
      //   CICD: message
      headerPattern: /^(?:([A-Z]{4})-(\d{2,3})|(HTFX)|(CICD)): (.+)$/,
      headerCorrespondence: ["code", "num", "htfx", "cicd", "subject"],
    },
  },

  rules: {
    // Total header length (industry standard ~72)
    "header-max-length": [2, "always", 72],

    // Require something after the ": "
    "subject-empty": [2, "never"],

    // No trailing period
    "subject-full-stop": [2, "never", "."],
  },
};
