function matrixToCArray(matrix, array_name, file_handle)

[rows, cols] = size(matrix);

fprintf(file_handle, 'const int %s_rows = %d;\n', array_name, rows);
fprintf(file_handle, 'const int %s_cols = %d;\n', array_name, cols);
fprintf(file_handle, 'double %s[%d][%d] = {\n', array_name, rows, cols);
for row=1:rows
    fprintf(file_handle, '  {');
    fprintf(file_handle, '%.20g, ', matrix(row,:));
    if row==rows
        fprintf(file_handle, '}\n');
    else
        fprintf(file_handle, '},\n');
    end
end
fprintf(file_handle, '};\n\n');