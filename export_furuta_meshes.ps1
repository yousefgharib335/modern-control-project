# Run with Windows PowerShell on a machine with Autodesk Inventor installed.
# Export source-local CAD surfaces in meters. Source documents are never saved.
$ErrorActionPreference = 'Stop'
$folder = Join-Path $PSScriptRoot 'cad\simscape-exports'
$parameters = Get-Content -LiteralPath (Join-Path $folder 'plant-properties.json') -Raw | ConvertFrom-Json
$cad = New-Object -ComObject Inventor.Application
$priorSilent = $cad.SilentOperation
$openedDocument = $null
$exported = @{}
try {
    $cad.SilentOperation = $true
    $translator = $cad.ApplicationAddIns.ItemById('{533E9A98-FC3B-11D4-8E7E-0010B541CD80}')
    foreach ($part in $parameters.parts) {
        $sourcePath = $part.source
        if (-not [IO.Path]::IsPathRooted($sourcePath)) {
            $sourcePath = Join-Path $PSScriptRoot $sourcePath
        }
        $sourcePath = (Resolve-Path -LiteralPath $sourcePath).Path
        $meshName = $part.mesh
        if ($exported.ContainsKey($meshName)) {
            if ($exported[$meshName] -ne $sourcePath) { throw "Conflicting source for $meshName." }
            continue
        }
        $document = $null
        foreach ($existing in $cad.Documents) {
            if ($existing.FullFileName -eq $sourcePath) { $document = $existing; break }
        }
        if ($null -eq $document) {
            $document = $cad.Documents.Open($sourcePath, $false)
            $openedDocument = $document
        }
        $context = $cad.TransientObjects.CreateTranslationContext()
        $context.Type = 13059 # kFileBrowseIOMechanism
        $options = $cad.TransientObjects.CreateNameValueMap()
        if (!$translator.HasSaveCopyAsOptions($document, $context, $options)) {
            throw "STL translator options unavailable for $($part.name)."
        }
        $options.Value('ExportUnits') = 6 # meters
        $options.Value('OutputFileType') = 0 # binary STL
        $options.Value('ExportFileStructure') = 0 # one file
        $options.Value('Resolution') = 0 # high
        $medium = $cad.TransientObjects.CreateDataMedium()
        $medium.FileName = Join-Path $folder $meshName
        $translator.SaveCopyAs($document, $context, $options, $medium)
        if (!(Test-Path -LiteralPath $medium.FileName)) {
            throw "Translator did not create $($medium.FileName)."
        }
        Get-Item -LiteralPath $medium.FileName | Select-Object Name, Length
        $exported[$meshName] = $sourcePath
        if ($null -ne $openedDocument) { $openedDocument.Close($true); $openedDocument = $null }
    }
} finally {
    if ($null -ne $openedDocument) { $openedDocument.Close($true) }
    $cad.SilentOperation = $priorSilent
}
